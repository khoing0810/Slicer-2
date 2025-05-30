#ifndef BEAM_PARSER_H
#define BEAM_PARSER_H
#include "common_parser.h"

namespace ORNL
{
    /*!
     * \class BeamParser
     * \brief This class implements the GCode parsing configuration for the
     * Beam 3D printer(s). \note The current commands that this class
     * implements are:
     *          - M Commands:
     *              - M110, M111
     */
    class BeamParser : public CommonParser
    {
        public:
            //! \brief Standard constructor that specifies meta type and whether or not
            //! to alter layers for minimal layer time
            //! \param meta GcodeMeta struct that includes information about units and
            //! delimiters
            //! \param allowLayerAlter bool to determinw whether or not to alter layers to
            //! adhere to minimal layer times
            //! \param lines Original lines of the gcode file
            //! \param upperLines Uppercase lines of the original used for ease of parsing/comparison
            BeamParser(GcodeMeta meta, bool allowLayerAlter, QStringList& lines, QStringList& upperLines);

            //! \brief Function to initialize syntax specific handlers
            virtual void config();

        protected:
            //! \brief Handler for 'M110' command for turning on extruder
            //! \param params Accepted by function for formatting check, but are not
            //! used for the command
            virtual void M110Handler(QVector<QString> params);

            //! \brief Handler for 'M111' command for turning off extruder
            //! \param params Accepted by function for formatting check, but are not
            //! used for the command
            virtual void M111Handler(QVector<QString> params);

    };
}

#endif // BEAM_PARSER_H
