#include "paparazzi.h"

using namespace Tangram;

#define AA_SCALE 2.0
#define MAX_WAITING_TIME 100.0

#if PLATFORM_LINUX
#include "platform_linux.h"
std::shared_ptr<LinuxPlatform> platform;
#elif PLATFORM_OSX
#include "platform_osx.h"
std::shared_ptr<OSXPlatform> platform;
#endif

#include "hash-library/md5.h"

#include <functional>
#include <csignal>
#include <fstream>
#include <regex>
#include <sstream>
#include <curl/curl.h>
#include "glm/trigonometric.hpp"

const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
const headers_t::value_type PNG_MIME{"Content-type", "image/png"};
const headers_t::value_type TXT_MIME{"Content-type", "text/plain;charset=utf-8"};

unsigned long long timeStart;

double getTime() {
    static struct timeval tv;

    gettimeofday(&tv, NULL);
    return ((unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000) * 0.001;
}

Paparazzi::Paparazzi()
  : m_scene("scene.yaml"),
    m_lat(0.0),
    m_lon(0.0),
    m_zoom(0.0f),
    m_rotation(0.0f),
    m_tilt(0.0),
    m_width(100),
    m_height(100) {

    m_glContext = std::make_unique<HeadlessContext>();
    if (!m_glContext->init()) {
        throw std::runtime_error("Could not initialize GL context");
    }
    m_glContext->resize(m_width, m_height);
    if (!m_glContext->makeCurrent()) {
        throw std::runtime_error("Could not activate GL context");
    }

    // Initialize Platform
    UrlClient::Options urlClientOptions;
    urlClientOptions.numberOfThreads = 10;

    platform = std::make_shared<LinuxPlatform>(urlClientOptions);

    timeStart = getTime();

    m_map = std::unique_ptr<Tangram::Map>(new Tangram::Map(platform));
    m_map->loadSceneAsync(m_scene.c_str());
    m_map->setupGL();
    m_map->setPixelScale(AA_SCALE);
    m_map->resize(m_width*AA_SCALE, m_height*AA_SCALE);
    update();
    
    //m_glContext->setScale(AA_SCALE);

    setSize(m_width, m_height, 1.0);


    Tangram::setDebugFlag(DebugFlags::tile_bounds, true);
}

Paparazzi::~Paparazzi() {
}

void Paparazzi::setSize (const int &_width, const int &_height, const float &_density) {
    if (_density*_width == m_width && _density*_height == m_height &&
        _density*AA_SCALE == m_map->getPixelScale()) { return; }
    
    m_width = _width*_density;
    m_height = _height*_density;

    // Setup the size of the image
    if (_density*AA_SCALE != m_map->getPixelScale()) {
        m_map->setPixelScale(_density*AA_SCALE);
    }
    m_map->resize(m_width*AA_SCALE, m_height*AA_SCALE);
    update();

    m_glContext->resize(m_width, m_height);
}

void Paparazzi::setZoom(const float &_zoom) {
    if (_zoom == m_zoom) { return; }
    m_zoom = _zoom;
    m_map->setZoom(_zoom);
    update();
}

void Paparazzi::setTilt(const float &_deg) {
    if (_deg == m_tilt) { return; }
    m_tilt = _deg;
    m_map->setTilt(glm::radians(m_tilt));
    update();
}
void Paparazzi::setRotation(const float &_deg) {
    if (_deg == m_rotation) { return; }
    
    m_rotation = _deg;
    m_map->setRotation(glm::radians(m_rotation));
    update();
}

void Paparazzi::setPosition(const double &_lon, const double &_lat) {
    if (_lon == m_lon && _lat == m_lat) { return; }
    
    m_lon = _lon;
    m_lat = _lat;
    m_map->setPosition(m_lon, m_lat);
    update();
}

void Paparazzi::setScene(const std::string &_url) {
    if (_url == m_scene) { return; }
    m_scene = _url;
    m_map->loadSceneAsync(m_scene.c_str());
    update();
}

void Paparazzi::setSceneContent(const std::string &_yaml_content) {
    MD5 md5;
    std::string md5_scene =  md5(_yaml_content);

    if (md5_scene == m_scene) { return; }
    m_scene = md5_scene;

    // TODO:
    //    - This is waiting for LoadSceneConfig to be implemented in Tangram::Map
    //      Once that's done there is no need to save the file.
    std::string name = "cache/"+md5_scene+".yaml";
    std::ofstream out(name.c_str());
    out << _yaml_content.c_str();
    out.close();

    m_map->loadSceneAsync(name.c_str());
    update();
}

void Paparazzi::update () {
    double startTime = getTime();
    float delta = 0.0;

    bool bFinish = false;
    while (delta < MAX_WAITING_TIME && !bFinish) {
        // Update Network Queue
        bFinish = m_map->update(10.);
        delta = float(getTime() - startTime);
        if (bFinish) {
            logMsg("Tangram::Update: Finish!\n");
        }
    }
    logMsg("Paparazzi::Update: Done waiting...\n");
}

struct coord_s {
    /** @brief coordinate x or column value */
    uint32_t x;
    /** @brief coordinate y or row value */
    uint32_t y;
    /** @brief coordinate z or zoom value */
    uint32_t z;
};

static double radians_to_degrees(double radians) {
    return radians * 180 / M_PI;
}

static double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180;
}

// http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
// TODO make output into point
void coord_to_lnglat(coord_s *coord, double *out_lng_deg, double *out_lat_deg) {
    double n = pow(2, coord->z);
    double lng_deg = coord->x / n * 360.0 - 180.0;
    double lat_rad = atan(sinh(M_PI * (1 - 2 * coord->y / n)));
    double lat_deg = radians_to_degrees(lat_rad);
    *out_lng_deg = lng_deg;
    *out_lat_deg = lat_deg;
}

// http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
// make input point
void lnglat_to_coord(double lng_deg, double lat_deg, int zoom, coord_s *out) {
    double lat_rad = degrees_to_radians(lat_deg);
    double n = pow(2.0, zoom);
    out->x = (lng_deg + 180.0) / 360.0 * n;
    out->y = (1.0 - log(tan(lat_rad) + (1 / cos(lat_rad))) / M_PI) / 2.0 * n;
    out->z = zoom;
}

// prime_server stuff
worker_t::result_t Paparazzi::work (const std::list<zmq::message_t>& job, void* request_info){
    // false means this is going back to the client, there is no next stage of the pipeline
    worker_t::result_t result{false, {}, ""};

    // This type differs per protocol hence the void* fun
    auto& info = *static_cast<http_request_info_t*>(request_info);

    // Try to generate a response 
    http_response_t response;
    try {
        // double start_call = getTime();

        //TODO: 
        //   - actually use/validate the request parameters
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()),
                                                   job.front().size());

        if (request.path == "/check") {
            // ELB check
          response = http_response_t(200, "OK", "OK", headers_t{CORS, TXT_MIME}, "HTTP/1.1");
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }
        
        //  SCENE
        //  ---------------------
        auto scene_itr = request.query.find("scene");
        if (scene_itr == request.query.cend() || scene_itr->second.size() == 0) {
            // If there is NO SCENE QUERY value 
            if (request.body.empty()) 
                // if there is not POST body content return error...
                throw std::runtime_error("scene is required punk");

            // ... other whise load content
            setSceneContent(request.body);
            // The size of the custom scene is unique enough
            result.heart_beat = std::to_string(request.body.size());
        }
        else {
            // If there IS a SCENE QUERRY value load it
            setScene(scene_itr->second.front());
            result.heart_beat = scene_itr->second.front();
        }

        bool size_and_pos = true;
        float pixel_density = 1.0f;

        //  SIZE
        auto width_itr = request.query.find("width");
        if (width_itr == request.query.cend() || width_itr->second.size() == 0)
            size_and_pos = false;
        auto height_itr = request.query.find("height");
        if (height_itr == request.query.cend() || height_itr->second.size() == 0)
            size_and_pos = false;
        auto density_itr = request.query.find("density");
        if (density_itr != request.query.cend() && density_itr->second.size() > 0)
            pixel_density = fmax(1.,std::stof(density_itr->second.front()));

        //  POSITION
        auto lat_itr = request.query.find("lat");
        if (lat_itr == request.query.cend() || lat_itr->second.size() == 0)
            size_and_pos = false;
        auto lon_itr = request.query.find("lon");
        if (lon_itr == request.query.cend() || lon_itr->second.size() == 0)
            size_and_pos = false;
        auto zoom_itr = request.query.find("zoom");
        if (zoom_itr == request.query.cend() || zoom_itr->second.size() == 0)
            size_and_pos = false;
            

        if (size_and_pos) {
            // Set Map and OpenGL context size
            setSize(std::stoi(width_itr->second.front()),
                    std::stoi(height_itr->second.front()),
                    pixel_density);
            setPosition(std::stod(lon_itr->second.front()),
                        std::stod(lat_itr->second.front()));
            setZoom(std::stof(zoom_itr->second.front()));

        } else {
            // Parse tile url
            const std::regex re("\\/(\\d*)\\/(\\d*)\\/(\\d*)\\.png");
            std::smatch match;

            if (std::regex_search(request.path, match, re) && match.size() == 4) {
                setSize(256,256, pixel_density);

                int tile_coord[3] = {0,0,0};
                for (int i = 0; i < 3; i++) {
                    std::istringstream cur(match.str(i+1));
                    cur >> tile_coord[i];
                }
                coord_s tile;
                tile.z = tile_coord[0];
                setZoom(tile.z);

                tile.x = tile_coord[1];
                tile.y = tile_coord[2];

                double n = pow(2, tile.z);
                double lng_deg = (tile.x + 0.5) / n * 360.0 - 180.0;
                double lat_rad = atan(sinh(M_PI * (1 - 2 * (tile.y + 0.5) / n)));
                double lat_deg = radians_to_degrees(lat_rad);
                    
                setPosition(lng_deg, lat_deg);
            }
            else {
                throw std::runtime_error("not enought data to construct image");
            }
        }

        //  OPTIONAL tilt and rotation
        //  ---------------------
        auto tilt_itr = request.query.find("tilt");
        if (tilt_itr != request.query.cend() && tilt_itr->second.size() != 0) {
            // If TILT QUERRY is provided assigned ...
            setTilt(std::stof(tilt_itr->second.front()));
        }
        else {
            // othewise use default (0.)
            setTilt(0.0f);
        }

        auto rotation_itr = request.query.find("rotation");
        if (rotation_itr != request.query.cend() && rotation_itr->second.size() != 0) {
            // If ROTATION QUERRY is provided assigned ...
            setRotation(std::stof(rotation_itr->second.front()));
        }
        else {
            // othewise use default (0.)
            setRotation(0.0f);
        }

        // Time to render
        //  ---------------------
        std::string image;
        if (m_map) {
            update();

            // Render Tangram Scene
            //m_glContext->bind();
            m_map->render();
            //m_glContext->unbind();
   
            // Once the main FBO is draw take a picture
            //m_glContext->getPixelsAsString(image);
            // double total_time = getTime()-start_call;
            // LOG("TOTAL CALL: %f", total_time);
            // LOG("TOTAL speed: %f millisec per pixel", (total_time/((m_width * m_height)/1000.0)));
        }

        response = http_response_t(200, "OK", image, headers_t{CORS, PNG_MIME}, "HTTP/1.1");
    }
    catch(const std::exception& e) {
        response = http_response_t(400, "Bad Request", e.what(), headers_t{CORS}, "HTTP/1.1");
    }

    response.from_info(info);
    result.messages.emplace_back(response.to_string());
    
    return result;
}

void Paparazzi::cleanup () {

}
