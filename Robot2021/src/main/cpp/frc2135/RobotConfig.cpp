/*
 * RobotConfig.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Administrator
 */

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <map>
#include <algorithm>
#include <unistd.h>        // gethostname
#include <string.h>        // strncmp

#include <frc2135/RobotConfig.h>

//////////////////////////////////////////////////////////

namespace frc2135
{

RobotConfig *RobotConfig::currentConfig = nullptr;

//////////////////////////////////////////////////////////

void trimWhitespace(std::string& line)
{
    line.erase(std::remove_if( line.begin(), line.end(), [](char c) { return (c =='\r' || c =='\t' || c == ' ' || c == '\n');}), line.end() );
}

////////////////////////////////////////////////////////////////////////////////

RobotConfig::RobotConfig()
{
    std::printf("2135: RobotConfig -- LoadConfig Started  --\n");
    LoadConfig();
    std::printf("2135: RobotConfig -- LoadConfig Finished --\n");
    std::printf("2135: RobotConfig -- DumpConfig Started  --\n");
    DumpConfig();
    std::printf("2135: RobotConfig -- DumpConfig Finished --\n");
}

RobotConfig::~RobotConfig()
{

}

////////////////////////////////////////////////////////////////////////////////

RobotConfig *RobotConfig::GetInstance()
{
    if (RobotConfig::currentConfig == nullptr)
    {
        std::printf("2135: RobotConfig -- Creating new RobotConfig object\n");
        RobotConfig::currentConfig = new RobotConfig();
    }

    return RobotConfig::currentConfig;
}

////////////////////////////////////////////////////////////////////////////////

void RobotConfig::GetConfigFileName(std::string& fileName)
{
    // Initialize with the absolute path to the home directory
    fileName = "/home/lvuser/deploy/";

    // Get the host name of the roboRIO
    const size_t NAMEBUFSIZE = 12;
    char nameBuf[NAMEBUFSIZE+1];

    memset(nameBuf, 0, sizeof(nameBuf));
    gethostname(nameBuf, NAMEBUFSIZE);
    nameBuf[NAMEBUFSIZE] = '\0';

    // Extract the robot number from the hostname (assumed to be of the form roboRIO-<4-digit-number>-FRC)
    char numBuf[5];

    strncpy(numBuf, nameBuf+8, 4);
    numBuf[4] = '\0';

    // Add the robot number to the file name
    fileName += numBuf;

    // Add the configuration file name suffix
    fileName += "_configuration.txt";
}

////////////////////////////////////////////////////////////////////////////////

bool RobotConfig::LoadConfig()
{
    std::string fileName;

    GetConfigFileName(fileName);
    std::ifstream configFile(fileName.c_str());
    if (configFile.good() == false)
    {
        std::printf("2135: ERROR: RobotConfig -- No such file %s\n", fileName.c_str());
        return false;
    }

    m_configMap.clear();

    std::printf("2135: RobotConfig -- Loading File %s\n", fileName.c_str());
    while (configFile.eof() == false)
    {
        std::string name;
        std::string valueStr;
        std::string delimiter = "=";
        std::string line;

        while(getline(configFile, line))
        {
            name = "";            // Reset

            trimWhitespace(line);
            if (line[0] == '#')    // Skipping comment line
                continue;

            size_t pos = 0;
            if ((pos = line.find(delimiter)) != std::string::npos)
            {
                name = line.substr(0, pos);
                trimWhitespace(name);
                line.erase(0, pos + delimiter.length());
                trimWhitespace(line);
                valueStr = line;
            }

            if (!name.empty())
                m_configMap[name] = valueStr;

            if (configFile.bad() == true)
            {
                std::printf("2135: ERROR: RobotConfig -- configFile %s bad\n", fileName.c_str());
                return false;
            }
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////

// will return false only if there is no value found or specified (default or from file)

bool RobotConfig::GetValueAsString(const std::string& name, std::string& valueStr, std::string defaultStr)
{
    bool rtnStatus = false;

    valueStr = m_configMap[name];

    if (!valueStr.empty())
    {
         rtnStatus = true;
    }
    else  // this name was not found in m_configMap, so add it using defaultValue.
    {
        if (!defaultStr.empty())
        {
            valueStr = defaultStr;
            m_configMap[name] = valueStr;
            std::printf("2135: WARNING: RobotConfig -- %s NOT found - using default value %s\n", name.c_str(), defaultStr.c_str());
            rtnStatus = true;
        }
        else
            std::printf("2135: ERROR: RobotConfig -- %s NOT found and no default value\n", name.c_str());
    }

    return rtnStatus;
}

////////////////////////////////////////////////////////////////////////////////

// Returns true if successful, puts value in valueInt.

bool RobotConfig::GetValueAsInt(const std::string& name, int& valueInt, int defaultInt)
{
    bool rtnStatus = false;

    std::string valueStr = m_configMap[name];

    if (!valueStr.empty())
    {
        valueInt = atoi(valueStr.c_str());  // convert to int
        rtnStatus = true;
    }
    else  // this name is not found in m_configMap, so add it if there is a default specified
    {
        if (defaultInt != DUMMY_DEFAULT_INT)
        {
            valueInt = defaultInt;
            std::string valueStr = std::to_string(defaultInt);
            m_configMap[name] = valueStr;
            std::printf("2135: WARNING: RobotConfig -- %s not found - using default value %d\n", name.c_str(), defaultInt);
            rtnStatus = true;
        }
        else
            std::printf("2135: ERROR: RobotConfig -- %s not found and no default value\n", name.c_str());
    }

    return rtnStatus;
}

////////////////////////////////////////////////////////////////////////////////

// Returns true if given name is found in config map & puts its value in valueBool.
//     default bool if no default is specified  will be false

bool RobotConfig::GetValueAsBool(const std::string& name, bool& valueBool, bool defaultBool)
{
    bool rtnStatus = false;

    std::string valueStr = m_configMap[name];

    if (!valueStr.empty())
    {
        // Assumes that value string is always lowercase 'true' or '1'.
        if ((valueStr == "true") || (valueStr == "1"))
            valueBool = true;
        else
            valueBool = false;
        rtnStatus = true;
    }
    else  // not found in m_configMap, so use default.
    {
        valueBool = defaultBool;
        std::string defaultStr("false");
        if (defaultBool)
            defaultStr = "true";
        m_configMap[name] = defaultStr;
        std::printf("2135: WARNING: RobotConfig -- %s not found - using default value %d\n", name.c_str(), defaultBool);
        rtnStatus = true;
    }

    return rtnStatus;
}

////////////////////////////////////////////////////////////////////////////////

bool RobotConfig::GetValueAsFloat(const std::string& name, float& valueFloat, float defaultFloat)
{
    bool rtnStatus = false;

    std::string valueStr = m_configMap[name];

    if (!valueStr.empty())
    {
        valueFloat = atof(valueStr.c_str());  // convert stored value to float
        rtnStatus = true;
    }
    else
    {
        if (defaultFloat != DUMMY_DEFAULT_FLOAT)
        {
            valueFloat = defaultFloat;
            std::string defaultStr = std::to_string(defaultFloat);
            m_configMap[name] = defaultStr;
            std::printf("2135: WARNING: RobotConfig -- %s not found using default value %f\n", name.c_str(), defaultFloat);
            rtnStatus = true;
        }
        else
            std::printf("2135: ERROR: RobotConfig -- %s not found and no default value specified\n", name.c_str());
    }

    return rtnStatus;
}

////////////////////////////////////////////////////////////////////////////////

bool RobotConfig::GetValueAsDouble(const std::string& name, double& valueDouble, double defaultDouble)
{
    bool rtnStatus = false;

    std::string valueStr = m_configMap[name];

    if (!valueStr.empty())
    {
        valueDouble = atof(valueStr.c_str());  // convert stored value to double
        rtnStatus = true;
    }
    else
    {
        if (defaultDouble != DUMMY_DEFAULT_DOUBLE)
        {
            valueDouble = defaultDouble;
            std::string defaultStr = std::to_string(defaultDouble);
            m_configMap[name] = defaultStr;
            std::printf("2135: WARNING: RobotConfig -- %s not found - using default value %f\n", name.c_str(), defaultDouble);
            rtnStatus = true;
        }
        else
            std::printf("2135: ERROR: RobotConfig -- %s not found and no default value specified\n", name.c_str());
    }

    return rtnStatus;
}

////////////////////////////////////////////////////////////////////////////////

void RobotConfig::DumpConfig()
{
    for (std::map<std::string, std::string>::iterator itr = m_configMap.begin(); itr != m_configMap.end(); itr++)
    {
        std::cout << itr->first << ": " << itr->second << "\n";
    }

// This is testing the get functions with our dummy file. We can remove this once we are confident with the functions.

#if defined (CONFIG_TESTING)
    float valueFloat;
    if (GetValueAsFloat("AutonDriveSpeed", valueFloat))
        std::cout<<"Test - AutonDriveSpeed" <<" " <<valueFloat <<"\n";
    else
        std::printf("AutonDriveSpeed could not get float value.\n");

    int valueInt;
    if (GetValueAsInt("BlahBlah", valueInt))
        std::cout<<"Test - BlahBlah (as int)" <<" " <<valueInt <<"\n";
    else
        std::printf("BlahBlah could not get int value.\n");

    bool valueBool;
    if (GetValueAsBool("BlahBlah", valueBool))
        std::cout<<"Test - BlahBlah (as bool)" <<" " <<valueBool <<"\n";
    else
        std::printf("BlahBlah could not get bool value.\n");

    if (GetValueAsBool("NatureWalk", valueBool ))
        std::cout<<"Test - NatureWalk" <<" " <<valueBool <<"\n";
    else
        std::printf("NatureWalk could not get value.\n");

    if (GetValueAsFloat("DeprecatedClimber", valueFloat))
        std::cout<<"Test - DeprecatedClimber" <<" " <<valueFloat <<"\n";
    else
        std::printf("DeprecatedClimber could not get float value.\n");

    if (GetValueAsInt("ThingOne", valueInt))
        std::cout<<"Test - ThingOne" <<" " <<valueInt <<"\n";
    else
        std::printf("ThingOne could not get int value.\n");

    if (GetValueAsInt("ThingTwo", valueInt))
        std::cout<<"Test - ThingTwo" <<" " <<valueInt <<"\n";
    else
        std::printf("ThingTwo could not get int value.\n");

    if (GetValueAsBool("PeopleOut", valueBool))
        std::cout<<"Test - PeopleOut (as bool)" <<" " <<valueBool <<"\n";
    else
        std::printf("PeopleOut could not get bool value.\n");

    std::string valueString;
    if (GetValueAsString("PeopleOut", valueString))
        std::cout<<"Test - PeopleOut (as string)" <<" " <<valueString <<"\n";
    else
        std::printf("PeopleOut (as string) could not get value.\n");

    if (GetValueAsFloat("Keyboard", valueFloat, 22.5))
        std::cout<<"Test - Keyboard" << " " <<valueFloat <<"\n";
    else
        std::printf("Keyboard could not get float value.\n");

    // The expected value of Keyboard should be 22.5
    if (GetValueAsFloat("Keyboard", valueFloat, 56.7))
        std::cout<<"Test - Keyboard" << " " <<valueFloat <<"\n";
    else
        std::printf("Keyboard could not get float value.\n");

    if (GetValueAsFloat("Mouse", valueFloat))
        std::cout<<"Test - Mouse" << " " <<valueFloat <<"\n";
    else
        std::printf("Keyboard could not get float value.\n");

    if (GetValueAsInt("Fence", valueInt))
        std::cout<<"Test - Fence (as int)" <<" " <<valueInt <<"\n";
    else
        std::printf("Fence could not get int value.\n");

    if (GetValueAsInt("Gate", valueInt, 999999))
        std::cout<<"Test - Gate (int)" <<" " <<valueInt <<"\n";
    else
        std::printf("Gate could not get int value.\n");

    if (GetValueAsBool("Tea", valueBool))
        std::cout<<"Test - Tea (as bool)" <<" " <<valueBool <<"\n";
    else
        std::printf("Tea could not get bool value.\n");

    if (GetValueAsFloat("Chair",valueFloat, DUMMY_DEFAULT_FLOAT))
        std::cout << "Test - Chair (as float)" << " " << valueFloat << "\n";
    else
        std::printf("Chair could not get float value.\n");

    if (GetValueAsString("Sweatpants", valueString))
        std::cout<<"Test - Sweatpants (as string)" <<" " <<valueString <<"\n";
    else
        std::printf("Sweatpants (as string) could not get value.\n");
#endif

}

} /* namespace frc2135 */
