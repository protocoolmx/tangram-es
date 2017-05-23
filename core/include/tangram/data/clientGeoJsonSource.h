#pragma once

#include "data/tileSource.h"
#include "util/types.h"

#include <mutex>


namespace Tangram {

class Platform;

struct Properties;

struct ClientGeoJsonData;

class ClientGeoJsonSource : public TileSource {

public:

    ClientGeoJsonSource(std::shared_ptr<Platform> _platform, const std::string& _name,
            const std::string& _url, TileSource::SourceOptions _sourceOptions,
            bool generateCentroids);
    ~ClientGeoJsonSource();

    // http://www.iana.org/assignments/media-types/application/geo+json
    virtual const char* mimeType() const override { return "application/geo+json"; };

    // Add geometry from a GeoJSON string
    void addData(const std::string& _data, bool _generateCentroids);
    void addPoint(const Properties& _tags, LngLat _point);
    void addLine(const Properties& _tags, const Coordinates& _line);
    void addPoly(const Properties& _tags, const std::vector<Coordinates>& _poly,
            bool _generateCentroids = false);
    void generateLabelCentroidFeature();

    virtual void loadTileData(std::shared_ptr<TileTask> _task, TileTaskCb _cb) override;
    std::shared_ptr<TileTask> createTask(TileID _tileId, int _subTask) override;

    virtual void cancelLoadingTile(const TileID& _tile) override {};
    virtual void clearData() override;

protected:

    virtual std::shared_ptr<TileData> parse(const TileTask& _task,
                                            const MapProjection& _projection) const override;

    std::unique_ptr<ClientGeoJsonData> m_store;

    mutable std::mutex m_mutexStore;
    bool m_hasPendingData = false;

    std::shared_ptr<Platform> m_platform;

};

}
