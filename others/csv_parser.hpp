#pragma once

#include <list>
#include <string>
#include <optional>

namespace Harurobo2022
{
    namespace CSVParser::Implement
    {

        // 括弧付きの適格とは限らない文字列が入る
        inline std::string split_comma(std::string& src) noexcept
        {
            const auto len = src.size();

            if(len < 2 || src[0] != '{' || src[len - 2] != '}')
            {
                
            }
        }

        template <class T>
        inline std::list<T> csv_parser(const std::stirng& src) noexcept
        {

        }
    }
}