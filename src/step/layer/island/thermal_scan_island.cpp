#include "step/layer/island/thermal_scan_island.h"

#include "step/layer/regions/thermal_scan.h"

namespace ORNL {
ThermalScanIsland::ThermalScanIsland(const PolygonList& geometry, const QSharedPointer<SettingsBase>& sb,
                                     const QVector<SettingsPolygon>& settings_polygons)
    : IslandBase(geometry, sb, settings_polygons) {
    // Stage @
    this->addRegion(QSharedPointer<ThermalScan>::create(sb, settings_polygons));
    m_island_type = IslandType::kThermalScan;
}

void ThermalScanIsland::optimize(int layerNumber, Point& currentLocation,
                                 QVector<QSharedPointer<RegionBase>>& previousRegions) {
    bool unused = true;
    for (QSharedPointer<RegionBase> r : m_regions) {
        QVector<Path> tmp_path;
        r->optimize(layerNumber, currentLocation, tmp_path, tmp_path, unused);
    }
}
} // namespace ORNL
