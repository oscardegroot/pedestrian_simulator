#include "xml_reader.h"

void XMLReader::Read()
{

    std::string map_file = ros::package::getPath("pedestrian_simulator") + "/scenarios/" + CONFIG.scenario_file_;
    ROS_INFO_STREAM("XMLReader: Reading " << map_file << " for pedestrian positions.");
    Read(map_file);
}

void XMLReader::Read(const std::string &file_name)
{

    ROS_INFO("Reading map file");

    // If the path does not contain the package, add it
    std::string map_file;
    if (file_name.find("//pedestrian_simulator//") != std::string::npos)
        map_file = ros::package::getPath("pedestrian_simulator") + "/" + file_name;
    else
        map_file = file_name;

    // Read the file with the correct file extension
    ReadXML(map_file);
}

void XMLReader::GetPedestrians(std::vector<std::unique_ptr<Pedestrian>> &pedestrian_ptrs)
{
    pedestrian_ptrs.clear();
    for (auto &ped : pedestrians_)
    {
        pedestrian_ptrs.emplace_back();
        pedestrian_ptrs.back().reset((Pedestrian *)(&ped)); // Set the pointer as the address of the current ped
    }
}

void XMLReader::ReadXML(const std::string &file)
{
    rapidxml::file<> xmlFile(file.c_str()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    pedestrians_.clear();

    // Read tags
    for (rapidxml::xml_node<> *tag = doc.first_node("tag"); tag; tag = tag->next_sibling("tag"))
    {
        if (std::string(tag->first_attribute("type")->value()).compare("binomial") == 0)
            CONFIG.ped_type_ = PedestrianType::BINOMIAL;
        else if (std::string(tag->first_attribute("type")->value()).compare("gaussian") == 0)
            CONFIG.ped_type_ = PedestrianType::GAUSSIAN;

        if (std::string(tag->first_attribute("type")->value()).compare("static") == 0)
            CONFIG.static_ = true;

        if (std::string(tag->first_attribute("type")->value()).compare("velocity") == 0)
            CONFIG.ped_velocity_ = atof(tag->first_attribute("value")->value());
    }

    // try{
    // For all pedestrians in the file
    for (rapidxml::xml_node<> *ped = doc.first_node("pedestrian"); ped; ped = ped->next_sibling("pedestrian"))
    {
        rapidxml::xml_node<> *start_point = ped->first_node("start");

        // Create a pedestrian
        Waypoint new_waypoint(
            atof(start_point->first_attribute("x")->value()),
            atof(start_point->first_attribute("y")->value()));
        pedestrians_.emplace_back(new_waypoint);

        // Read all paths
        for (rapidxml::xml_node<> *path = ped->first_node("path"); path; path = path->next_sibling("path"))
        {
            pedestrians_.back().paths_.emplace_back();
            for (rapidxml::xml_node<> *point = path->first_node("point"); point; point = point->next_sibling("point"))
            {
                pedestrians_.back().paths_.back().emplace_back(Waypoint(
                    atof(point->first_attribute("x")->value()),
                    atof(point->first_attribute("y")->value())));
            }
        }
    }
}