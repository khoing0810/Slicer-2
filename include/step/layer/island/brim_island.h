#pragma once

#include "step/layer/island/island_base.h"

namespace ORNL {
/*!
 * \class BrimIsland
 * \brief Island that polymer builds use.
 */
class BrimIsland : public IslandBase {
  public:
    //! \brief Constructor
    //! \param geometry: the outlines
    //! \param sb: the settings
    //! \param settings_polygons: a vector of settings polygons to apply
    BrimIsland(const PolygonList& geometry, const QSharedPointer<SettingsBase>& sb,
               const QVector<SettingsPolygon>& settings_polygons);

    //! \brief Override from base. Filters down to individual regions to add
    //! travels and apply path modifiers
    void optimize(int layerNumber, Point& currentLocation,
                  QVector<QSharedPointer<RegionBase>>& previousRegions) override;
};
} // namespace ORNL
