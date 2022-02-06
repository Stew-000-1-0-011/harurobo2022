/*
* usb_can_node.cpp
*
*  Created on: Mar 4, 2018
*      Author: yusaku
*/

#include <ros/ros.h>

#include <string>
#include <thread>
#include <mutex>
#include <array>

#include <cstring>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <can_msgs/CanFrame.h>

using namespace std;
using namespace boost::asio;

/*
class UsbCanBridge
{
public:
    UsbCanBridge(void);
};
*/

class UsbCanNode
{
public:
    UsbCanNode(void);

    bool Initialize(void);

    bool Open(void);
    bool Close(void);

    bool Start(void);
    bool Stop(void);

    bool SetBaudRate(const int baud);

    void Dispose(void);

private:
    ros::NodeHandle _nh;

    //bool SendDataFrame(const uint32_t id, const uint8_t length, const uint8_t * const data);

    void canRxTask(void);
    void processRxFrame(const uint8_t * const str_buf, const uint16_t str_len);

    void canTxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    // return true if aborted
    // return false on success
    bool waitForStatus(void);

    void send(const uint8_t * const str);
    inline void send(const char * const str)
    {
        send((const uint8_t * const)str);
    }

    void onReceive(const boost::system::error_code& error);
    void receive(void);

    can_msgs::CanFrame can_rx_msg;
    ros::Publisher can_rx_pub;
    ros::Subscriber can_tx_sub;

    boost::asio::serial_port *_port;
    boost::asio::io_service *_io;
    boost::asio::io_service::work *_w;

    boost::asio::strand *_read_strand;
    boost::asio::strand *_write_strand;

    boost::thread_group _service_threads;

    boost::asio::streambuf _receive_buffer;

    int _status = 0;
    bool _status_changed = false;

    string _port_name;
    int _baud;
    int _baud_uart;	// this really needs to be deprecated

    static constexpr int RX_STR_SIZE = 32;
    uint8_t _rx_str_buf[RX_STR_SIZE];
    uint16_t _rx_str_len = 0;

    int _packet_received = 0;

    bool _is_open = false;

    //char rbuf[32];

    volatile bool _stop = false;
    std::mutex _mtx;

    //std::thread *can_rx_thread;

    static constexpr int CAN_MTU = 32;
};


UsbCanNode::UsbCanNode(void)
{
    auto nh_priv = ros::NodeHandle("~");
    if(!nh_priv.getParam("port", _port_name))
    {
        _port_name = "/dev/ttyUSB0";
        //ROS_ERROR("value for parameter port is invalid.");
        //exit(-1);
    }

    if(!nh_priv.getParam("baud", _baud))
    {
        _baud = 1000000;
        //ROS_ERROR("value for parameter baud is invalid.");
        //exit(-1);
    }

    if(!nh_priv.getParam("baud_uart", _baud_uart))
    {
        _baud_uart = 921600;
        //ROS_ERROR("value for parameter baud_uart is invalid.");
        //exit(-1);
    }

    this->_io = new io_service();
    this->_port = new serial_port(*_io);

    this->_w = new boost::asio::io_service::work(*_io);

    this->_read_strand = new boost::asio::strand(*_io);
    this->_write_strand = new boost::asio::strand(*_io);

    // 2 threads for read/write
    this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _io));
    this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _io));

    this->can_tx_sub = _nh.subscribe<can_msgs::CanFrame>("/can_tx", 10, &UsbCanNode::canTxCallback, this);
    this->can_rx_pub = _nh.advertise<can_msgs::CanFrame>("/can_rx", 1);

    //this->can_rx_thread = new std::thread(&UsbCanNode::canRxTask, this);
}

bool UsbCanNode::waitForStatus(void)
{
    while(!this->_status_changed)
    {
        if(this->_stop)
        {
            return true;
        }
    }

    this->_status_changed = false;

    return false;
}

bool UsbCanNode::Initialize(void)
{
    if(!this->_port->is_open())
    {
        ROS_ERROR("serial port is not open");
        return true;
    }

    // flush and close current connection
    //_port->write_some(buffer("\r", 1));
    this->send("\r");

    this->waitForStatus();
    if(this->_status)
    {
        // couldn't care less about the status
        //return true;
    }

    ROS_INFO("closing last session");

    //_port->write_some(buffer("C\r", 2));
    this->send("C\r");

    this->waitForStatus();
    if(this->_status)
    {
        // now i do
        return true;
    }

    return false;
}

bool UsbCanNode::Open(void)
{
    boost::system::error_code _error;

    if(this->_port->is_open())
    {
        ROS_ERROR("tried to open port while it's already open");

        return true;
    }

    this->_port->open(_port_name, _error);

    if(_error)
    {
        ROS_ERROR("failed to open port: %d", _error.value());

        return true;
    }

    _port->set_option(serial_port_base::baud_rate(_baud_uart));
    _port->set_option(serial_port_base::character_size(8));
    _port->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    _port->set_option(serial_port_base::parity(serial_port_base::parity::none));
    _port->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

    this->receive();

    ROS_INFO("successfully opened serial port");

    return false;
}

bool UsbCanNode::Close(void)
{
    boost::system::error_code _error;
    if(!this->_port->is_open())
    {
        // pretend nothing happend
        ROS_DEBUG("successfully closed serial port : port already closed");
        return false;
    }

    this->_port->cancel();

    this->_port->close(_error);

    if(_error)
    {
        // wtf
        ROS_ERROR("failed to CLOSE port: %d", _error.value());

        return true;
    }

    ROS_DEBUG("successfully closed serial port");
    return false;
}

bool UsbCanNode::Start(void)
{
    if(!this->_port->is_open())
    {
        ROS_ERROR("serial port is not open");
        return true;
    }

    //_port->write_some(buffer("O\r", 2));
    this->send("O\r");

    this->waitForStatus();
    if(this->_status)
    {
        this->_is_open = false;
        return true;
    }

    this->_is_open = true;
    return false;
}

bool UsbCanNode::Stop(void)
{
    if(!this->_port->is_open())
    {
        ROS_ERROR("serial port is not open");
        return true;
    }

    //_port->write_some(buffer("C\r", 2));
    this->send("C\r");
    this->_is_open = false;

    this->waitForStatus();

    bool result = this->_status;

    return result;
}

bool UsbCanNode::SetBaudRate(const int baud)
{
    //string brspec;

    if(!this->_port->is_open())
    {
        ROS_ERROR("serial port is not open");
        return true;
    }

    if(baud == 10000)
    {
        //_port->write_some(buffer("S0\r", 3));
        this->send((const uint8_t * const)"S0\r");
    }
    else if(baud == 20000)
    {
        //_port->write_some(buffer("S1\r", 3));
        this->send((const uint8_t * const)"S1\r");
    }
    else if(baud == 50000)
    {
        //_port->write_some(buffer("S2\r", 3));
        this->send((const uint8_t * const)"S2\r");
    }
    else if(baud == 100000)
    {
        //_port->write_some(buffer("S3\r", 3));
        this->send((const uint8_t * const)"S3\r");
    }
    else if(baud == 125000)
    {
        //_port->write_some(buffer("S4\r", 3));
        this->send((const uint8_t * const)"S4\r");
    }
    else if(baud == 250000)
    {
        //_port->write_some(buffer("S5\r", 3));
        this->send((const uint8_t * const)"S5\r");
    }
    else if(baud == 500000)
    {
        //_port->write_some(buffer("S6\r", 3));
        this->send((const uint8_t * const)"S6\r");
    }
    else if(baud == 800000)
    {
        //_port->write_some(buffer("S7\r", 3));
        this->send((const uint8_t * const)"S7\r");
    }
    else if(baud == 1000000)
    {
        //_port->write_some(buffer("S8\r", 3));
        this->send((const uint8_t * const)"S8\r");
    }
    else
    {
        return false;
    }

    this->waitForStatus();
    if(this->_status)
    {
        return true;
    }

    return false;
}

void UsbCanNode::processRxFrame(const uint8_t * const str_buf, const uint16_t str_len)
{
    static std::mutex m;
    std::lock_guard<std::mutex> _lock(m);

    uint8_t buf[32];
    int i = 0;

    buf[i] = str_buf[i];
    i++;

    // convert from ASCII (2nd character to end)
    for (i = 1; i < str_len; i++)
    {
        if(str_buf[i] >= 'a')
        {
            // lowercase letters
            buf[i] = str_buf[i] - 'a' + 10;
        }
        else if(str_buf[i] >= 'A')
        {
            // uppercase letters
            buf[i] = str_buf[i] - 'A' + 10;
        }
        else
        {
            // numbers
            buf[i] = str_buf[i] - '0';
        }
    }

    if (buf[0] == 't' || buf[0] == 'T')
    {
        // transmit data frame command
        this->can_rx_msg.is_rtr = false;
    }
    else if (buf[0] == 'r' || buf[0] == 'R')
    {
        // transmit remote frame command
        this->can_rx_msg.is_rtr = true;
    }
    else
    {
        // error, unknown command
        return;// -1;
    }

    uint8_t id_len;

    if (buf[0] == 't' || buf[0] == 'r')
    {
        this->can_rx_msg.is_extended = false;
        id_len = 3;
    }
    else if (buf[0] == 'T' || buf[0] == 'R')
    {
        this->can_rx_msg.is_extended = true;
        id_len = 8;
    }
    else
    {
        // error
        return;// -1;
    }
    this->can_rx_msg.is_error = false;

    i = 1;
    this->can_rx_msg.id = 0;
    while (i <= id_len)
    {
        this->can_rx_msg.id <<= 4;
        this->can_rx_msg.id += buf[i++];
    }

    this->can_rx_msg.dlc = buf[i++];
    if (this->can_rx_msg.dlc < 0 || this->can_rx_msg.dlc > 8)
    {
        return;// -1;
    }

    uint8_t j;
    for (j = 0; j < this->can_rx_msg.dlc; j++)
    {
        this->can_rx_msg.data[j] = (buf[i++] * 16) + buf[i++];
    }

    // send the message
    this->can_rx_pub.publish(this->can_rx_msg);
    //ROS_INFO("frame received");
}

void UsbCanNode::canTxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
    std::lock_guard<std::mutex> _lock(this->_mtx);

    uint8_t str_buf[32];

    int i = 0;

    int id_len;

    if(msg->is_rtr)
    {
        str_buf[i] = 'r';
    }
    else
    {
        str_buf[i] = 't';
    }

    if(msg->is_extended)
    {
        str_buf[i] -= 32;
        id_len = 8;
    }
    else
    {
        id_len = 3;
    }
    i++;

    int tmp = msg->id;
    for(int j = id_len; j > 0; j--)
    {
        str_buf[j] = tmp & 0x0f;
        tmp = tmp >> 4;
        i++;
    }

    // add DLC to buffer
    str_buf[i++] = msg->dlc;

    if(!msg->is_rtr)
    {
        // add data bytes
        for (int j = 0; j < msg->dlc; j++)
        {
            str_buf[i++] = (msg->data[j] >> 4);
            str_buf[i++] = (msg->data[j] & 0x0F);
        }
    }

    // convert to ASCII (2nd character to end)
    for (int j = 1; j < i; j++)
    {
        if (str_buf[j] < 0xA)
        {
            str_buf[j] += 0x30;
        }
        else
        {
            str_buf[j] += 0x37;
        }
    }

    // add carriage return (slcan EOL)
    str_buf[i++] = '\r';
    str_buf[i++] = '\0';

    //this->_port->write_some(buffer(str_buf, i));
    this->send(str_buf);
}

void UsbCanNode::send(const uint8_t * const str)
{
    static std::mutex m;
    std::lock_guard<std::mutex> _lock(m);

    //std::array<char, CAN_MTU> send_buf;

    try
    {
        this->_port->write_some(buffer(str, strlen((const char * const)str)));
    }
    catch(std::exception &ex)
    {
        ROS_WARN("an error occurred: %s", ex.what());
    }
}

void UsbCanNode::receive(void)
{
    //this->_io->reset();

    boost::asio::async_read(
            *_port,
            _receive_buffer,
            boost::asio::transfer_at_least(1), // receive at least one byte
            boost::bind(&UsbCanNode::onReceive, this, boost::asio::placeholders::error));
}

void UsbCanNode::onReceive(const boost::system::error_code& error)
{
    const std::string data(boost::asio::buffer_cast<const char*>(_receive_buffer.data()), _receive_buffer.size());
    _receive_buffer.consume(_receive_buffer.size());

    //ROS_INFO("onReceive()");

    for(const char c : data)
    {
        if(c == 0x07)
        {
            this->_status = -1;
            this->_status_changed = true;

            //ROS_INFO("S_Bell");
            _rx_str_len = 0;
        }
        else if(c == '\r')
        {
            if(_rx_str_len == 0 || _rx_str_buf[0] == 'z' || _rx_str_buf[0] == 'Z')
            {
                this->_status = 0;
                this->_status_changed = true;

                //ROS_INFO("S_OK");

                _rx_str_len = 0;
            }
            else
            {
                this->processRxFrame(_rx_str_buf, _rx_str_len);
                _rx_str_len = 0;
            }
        }
        else
        {
            _rx_str_buf[_rx_str_len++] = c;
            if(_rx_str_len > RX_STR_SIZE)
            {
                ROS_ERROR("eww");
            }
        }
    }

    receive();
}

void UsbCanNode::Dispose(void)
{
    //std::lock_guard<std::mutex> _lock(this->_mtx);

    this->_stop = true;
    ROS_INFO("stopping...");
    this->Close();
    //this->_service_thread->do_try_join_until(timespec(1000));
}

UsbCanNode *usbCanNode = nullptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usb_can_node");
    ROS_INFO("usb_can_node has started.");

    usbCanNode = new UsbCanNode();

    ROS_DEBUG("opening serial port...");
    if(usbCanNode->Open())
    {
        ROS_ERROR("failed to open serial port");
        exit(-1);
    }
    else
    {
        //ROS_INFO("opened ");
    }

    if(usbCanNode->Initialize())
    {
        ROS_ERROR("failed to initialize");
        exit(-1);
    }

    ROS_INFO("initialized");

    if(usbCanNode->SetBaudRate(1000000))
    {
        ROS_ERROR("failed to set baud rate");
    }
    else
    {
        ROS_INFO("baud rate set");
    }

    if(	usbCanNode->Start())
    {
        ROS_ERROR("failed to start session");
    }
    else
    {
        //ROS_INFO("baud rate set");
    }

    ros::spin();

    //usbCanNode->Dispose();



    ROS_INFO("usb_can_node has been terminated.");
}