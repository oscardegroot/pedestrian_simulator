#ifndef __XMLREADER_H__
#define __XMLREADER_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <stdlib.h> /* atoi */
#include <map>

#include "rapidxml_utils.hpp"

#include "lmpcc_tools/helpers.h"
#include "pedestrian.h"
#include "pedestrian_simulator/configuration.h"

class XMLReader
{

    /**
 * @brief Class for reading map files and converting it into necessary formats
 * @see types.h
 */
public:
    XMLReader()
    {
        Read();
    }

    std::vector<WaypointPedestrian> pedestrians_;

    void GetPedestrians(std::vector<std::unique_ptr<Pedestrian>> &pedestrian_ptrs);

public:
    /**
     * @brief Read an XML file with map data
     * 
     * @param file the file path to read from
     */
    void Read();

private:
    /**
     * @brief Read an XML file with map data
     * 
     * @param file the file path to read from
     */
    void Read(const std::string &file);

    void ReadXML(const std::string &file);
};

#endif // __XMLREADER_H__