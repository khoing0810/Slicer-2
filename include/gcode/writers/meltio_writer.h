#ifndef MELTIO_WRITER_H
#define MELTIO_WRITER_H

//! \file ornl_writer.h

#include "gcode/gcode_meta.h"
#include "gcode/writers/writer_base.h"
#include "managers/settings/settings_manager.h"

namespace ORNL {
/*!
 * \class MeltioWriter
 * \brief The gcode writer for the Meltio syntax
 */
class MeltioWriter : public WriterBase {
  public:
    //! \brief Constructor
    MeltioWriter(GcodeMeta meta, const QSharedPointer<SettingsBase>& sb);

    //! \brief Writes important settings info to the header for the operator to read
    QString writeSettingsHeader(GcodeSyntax syntax) override;

    //! \brief Writes initial setup instructions for the machine state
    QString writeInitialSetup(Distance minimum_x, Distance minimum_y, Distance maximum_x, Distance maximum_y,
                              int num_layers) override;

    //! \brief Writes G-Code to be executed at the start of the layer
    QString writeBeforeLayer(float min_z, QSharedPointer<SettingsBase> sb) override;

    //! \brief Writes G-Code to be executed each layer before each part
    QString writeBeforePart(QVector3D normal) override;

    //! \brief Writes G-Code to be exectued before each island
    QString writeBeforeIsland() override;

    //! \brief Writes G-Code to be executed at the start of each region
    QString writeBeforeRegion(RegionType type, int pathSize) override;

    //! \brief Writes G-Code to be executed at the start of each path
    QString writeBeforePath(RegionType type) override;

    //! \brief Writes G-Code for traveling between paths
    QString writeTravel(Point start_location, Point target_location, TravelLiftType lType,
                        QSharedPointer<SettingsBase> params) override;

    //! \brief Writes G-Code for line
    QString writeLine(const Point& start_point, const Point& target_point,
                      const QSharedPointer<SettingsBase> params) override;

    //! \brief Writes G-Code for arc
    QString writeArc(const Point& start_point, const Point& end_point, const Point& center_point, const Angle& angle,
                     const bool& ccw, const QSharedPointer<SettingsBase> params) override;

    //! \brief Writes G-Code to be executed after each path
    QString writeAfterPath(RegionType type) override;

    //! \brief Writes G-Code to be executed after each region
    QString writeAfterRegion(RegionType type) override;

    //! \brief Writes G-Code to be executed at the end of each island
    QString writeAfterIsland() override;

    //! \brief Writes G-Code to be executed at the end of each part each layer
    QString writeAfterPart() override;

    //! \brief Writes G-Code to be executed at the end of each layer
    QString writeAfterLayer() override;

    //! \brief Writes G-Code for shutting down the machine
    QString writeShutdown() override;

    //! \brief Write purge command
    QString writePurge(int RPM, int duration, int delay) override;

    //! \brief Writes G-Code for a pause, G4
    QString writeDwell(Time time) override;

  private:
    //! \brief Writes G-Code to enable the extruder
    QString writeExtruderOn(RegionType type, int rpm, int extruder_number);
    //! \brief Writes G-Code to disable the extruder
    QString writeExtruderOff(int extruder_number);

    //! \brief Writes gcode coordinates WXYZ for a move or travel to the destination point
    QString writeCoordinates(Point destination);

    //! \brief true if first travel, false for subsequent travels
    bool m_first_travel;
    //! \brief true if first printing segment, false for subsquent paths - needed for Spiralize
    bool m_first_print;
    //! \brief true is first print motion of the layer
    bool m_layer_start;

}; // class MeltioWriter
} // namespace ORNL
#endif // MELTIO_WRITER_H
