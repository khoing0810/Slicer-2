#include "step/layer/island/raft_island.h"

#include "managers/settings/settings_manager.h"
#include "step/layer/regions/raft.h"

namespace ORNL {
RaftIsland::RaftIsland(const PolygonList& geometry, const QSharedPointer<SettingsBase>& sb,
                       const QVector<SettingsPolygon>& settings_polygons)
    : IslandBase(geometry, sb, settings_polygons) {
    this->addRegion(QSharedPointer<Raft>::create(sb, settings_polygons));
    m_island_type = IslandType::kRaft;
}

void RaftIsland::optimize(int layerNumber, Point& currentLocation,
                          QVector<QSharedPointer<RegionBase>>& previousRegions) {
    bool unused = true;
    for (QSharedPointer<RegionBase> r : m_regions) {
        QVector<Path> tmp_path;
        r->optimize(layerNumber, currentLocation, tmp_path, tmp_path, unused);
    }
}
} // namespace ORNL
