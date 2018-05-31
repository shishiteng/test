/*
 * snTool.h
 *
 *  Created on: Apr 28, 2017
 *      Author: vip2
 */

#ifndef SNTOOL_H_
#define SNTOOL_H_

#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <fstream>

namespace snTool
{
/**
 * 将字符串的十六进制装换成十进制
 * @param hexStr
 */
inline long hex2int(const std::string &hexStr)
{
    char *offset;
    if (hexStr.length() > 2)
    {
        if (hexStr[0] == '0' && hexStr[1] == 'x')
        {
            return strtol(hexStr.c_str(), &offset, 0);
        }
    }
    return strtol(hexStr.c_str(), &offset, 16);
}

/**
 * 读取二进制文件
 * @param filename 文件名
 * @param data
 * @param byte
 */
template <class T>
void readBinaryFile(std::string filename, T **data, const int byte)
{
    if (*data == NULL)
    {
        *data = (T *)malloc(sizeof(T) * byte);
    }

    FILE *fp = fopen(filename.c_str(), "rb");
    fread((void *)*data, sizeof(T), byte, fp);
    fclose(fp);
}

} // namespace snTool

#endif /* SNTOOL_H_ */
