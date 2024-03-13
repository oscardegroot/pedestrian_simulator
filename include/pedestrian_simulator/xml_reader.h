#ifndef __XMLREADER_H__
#define __XMLREADER_H__

#include <pedestrians/pedestrian.h>
#include <pedestrian_simulator/spawn_randomizer.h>
#include <pedestrian_simulator/types.h>

#include <vector>
#include <string>
#include <memory>

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

    std::vector<std::unique_ptr<Pedestrian>> pedestrians_;
    std::vector<StaticObstacle> static_obstacles_;
    std::vector<SpawnRandomizer> spawn_randomizers_;

    std::vector<bool> is_random_;

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