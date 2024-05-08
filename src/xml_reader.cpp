#include <pedestrian_simulator/xml_reader.h>

#include <pedestrian_simulator/configuration.h>
#include <rapidxml_utils.hpp>

#include <ros_tools/paths.h>
#include <ros_tools/logging.h>

#include <stdlib.h> /* atoi */
#include <map>

void XMLReader::Read()
{

    std::string map_file = getPackagePath("pedestrian_simulator") + "/scenarios/" + CONFIG.scenario_file_;
    LOG_VALUE("Pedestrian Scenario", map_file);
    Read(map_file);
}

void XMLReader::Read(const std::string &file_name)
{

    LOG_INFO("Reading map file");

    // If the path does not contain the package, add it
    std::string map_file;
    if (file_name.find("//pedestrian_simulator//") != std::string::npos)
        map_file = getPackagePath("pedestrian_simulator") + "/" + file_name;
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

    // If there is a general random description, read it
    for (rapidxml::xml_node<> *tag = doc.first_node("random"); tag; tag = tag->next_sibling("random"))
    {
        spawn_randomizers_.emplace_back();
        spawn_randomizers_.back().ReadFrom(tag);
    }

    // Read tags
    for (rapidxml::xml_node<> *tag = doc.first_node("tag"); tag; tag = tag->next_sibling("tag"))
    {
        if (std::string(tag->first_attribute("type")->value()).compare("binomial") == 0)
            CONFIG.ped_type_ = PedestrianType::BINOMIAL;
        else if (std::string(tag->first_attribute("type")->value()).compare("gaussian") == 0)
            CONFIG.ped_type_ = PedestrianType::GAUSSIAN;
        else if (std::string(tag->first_attribute("type")->value()).compare("social") == 0)
            CONFIG.ped_type_ = PedestrianType::SOCIAL;

        if (std::string(tag->first_attribute("type")->value()).compare("static") == 0)
            CONFIG.static_ = true;

        if (std::string(tag->first_attribute("type")->value()).compare("uncertainty") == 0)
        {
            double noise = atof(tag->first_attribute("value")->value());
            CONFIG.process_noise_ = std::vector<double>({noise, noise});
        }
        if (std::string(tag->first_attribute("type")->value()).compare("velocity") == 0)
            CONFIG.ped_velocity_ = atof(tag->first_attribute("value")->value());
    }
    auto *random_pedestrian_tag = doc.first_node("random_pedestrians");
    if (random_pedestrian_tag)
    {
        int num_peds = atoi(random_pedestrian_tag->first_attribute("value")->value());
        for (int i = 0; i < num_peds; i++)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new Pedestrian(Waypoint(0., 0.), 0.));
            is_random_.push_back(true);
        }
    }

    // For all pedestrians in the file
    for (rapidxml::xml_node<> *ped = doc.first_node("pedestrian"); ped; ped = ped->next_sibling("pedestrian"))
    {
        rapidxml::xml_node<> *start_point = ped->first_node("start");

        bool random = false;
        for (rapidxml::xml_node<> *tag = ped->first_node("tag"); tag; tag = tag->next_sibling("tag"))
        {
            if (std::string(tag->first_attribute("type")->value()).compare("random") == 0)
            {
                random = true;
                break;
            }
        }

        if (random)
        {
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new Pedestrian(Waypoint(0., 0.), 0.));
            is_random_.push_back(true);
        }
        else
        {
            is_random_.push_back(false);
            // Create a pedestrian
            Waypoint new_waypoint(
                atof(start_point->first_attribute("x")->value()),
                atof(start_point->first_attribute("y")->value()));
            pedestrians_.emplace_back();
            pedestrians_.back().reset(new Pedestrian(new_waypoint, 0.));

            // Read all paths
            for (rapidxml::xml_node<> *path = ped->first_node("path"); path; path = path->next_sibling("path"))
            {
                // pedestrians_.back().paths_.emplace_back();
                for (rapidxml::xml_node<> *point = path->first_node("point"); point; point = point->next_sibling("point"))
                {
                    // pedestrians_.back().paths_.back().emplace_back(Waypoint(
                    pedestrians_.back()->goal_ = Waypoint(
                        atof(point->first_attribute("x")->value()),
                        atof(point->first_attribute("y")->value()));
                }
            }
        }
    }

    // For all pedestrians in the file
    for (rapidxml::xml_node<> *obs = doc.first_node("obstacle"); obs; obs = obs->next_sibling("obstacle"))
    {
        static_obstacles_.emplace_back(atof(obs->first_attribute("x1")->value()),
                                       atof(obs->first_attribute("y1")->value()),
                                       atof(obs->first_attribute("x2")->value()),
                                       atof(obs->first_attribute("y2")->value()));
    }
}