/*

In(Node) -> message
Node(message) -> message
message -> Out(Node)

In(Joy) -> buttons

ManualCommander(buttons) ->
        under_carriage_commands,
        collector_commands -> Out(UsbCan),
        wheel_lift_commands -> Out(UsbCan),
        emergency_commands -> Out(UsbCan)

UnderCarriage4Wheel(under_carriage_commands) -> drive_motors_vela -> Out(UsbCan)

In(UsbCan) -> odometory

CalcPos(odmetory) -> position

OrbitManager(pos) ->
        next_pos,
        commands




*/