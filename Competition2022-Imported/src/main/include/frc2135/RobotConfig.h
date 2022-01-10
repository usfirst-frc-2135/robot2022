/*
 * RobotConfig.h
 *
 *  Created on: Jan 23, 2020
 *      Author: Administrator
 */

#pragma once

#include <cfloat>
#include <climits>
#include <map>
#include <stdlib.h>
#include <string>
#include <vector>

static const int DUMMY_DEFAULT_INT = INT_MAX;
static const float DUMMY_DEFAULT_FLOAT = FLT_MAX;
static const double DUMMY_DEFAULT_DOUBLE = DBL_MAX;

namespace frc2135
{
    class RobotConfig
    {
    private:
        RobotConfig();
        virtual ~RobotConfig();

        static void GetConfigFileName(std::string &fileName);

        static RobotConfig *currentConfig;
        std::map<std::string, std::string> m_configMap;

    public:
        static RobotConfig *GetInstance();
        bool LoadConfig();
        bool GetValueAsString(const std::string &name, std::string &valueString, std::string defaultValue = "");
        bool GetValueAsInt(const std::string &name, int &valueInt, int defaultInt = DUMMY_DEFAULT_INT);
        bool GetValueAsBool(const std::string &name, bool &valueBool, bool defaultBool = false);
        bool GetValueAsFloat(const std::string &name, float &valueFloat, float defaultFloat = DUMMY_DEFAULT_FLOAT);
        bool GetValueAsDouble(
            const std::string &name,
            double &valueDouble,
            double defaultDouble = DUMMY_DEFAULT_DOUBLE);
        void DumpConfig();
    };

} /* namespace frc2135 */
