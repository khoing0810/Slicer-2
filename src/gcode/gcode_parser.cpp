#if 0
    #include "gcode/gcode_parser.h"

    #include "QDebug"
    #include "QIODevice"
    #include "QVariant"
    #include "QVector"
    #include "QtGlobal"
    #include "exceptions/exceptions.h"
    #include "gcode/gcode_command.h"
    #include "gcode/gcode_meta.h"
    #include "gcode/parsers/cincinnati_parser.h"
    #include "gcode/parsers/common_parser.h"

    #include <functional>

namespace ORNL
{
    GcodeParser::GcodeParser()
        : m_current_parser(
              nullptr,
              std::bind(&GcodeParser::freeParser, this, std::placeholders::_1))
    {
        selectParser(GcodeSyntax::kCommon);
    }

    GcodeParser::GcodeParser(GcodeSyntax id)
        : m_current_parser(
              nullptr,
              std::bind(&GcodeParser::freeParser, this, std::placeholders::_1))
    {
        selectParser(id);
    }

    GcodeCommand GcodeParser::parseLine(const QString& line)
    {
        if(m_current_parser == nullptr)
        {
            throwParserNotSetException();
        }
        return m_current_parser->parseCommand(line);
    }

    QPair< GcodeCommand, GcodeCommand > GcodeParser::updateLine(
        const QString& line,
        int line_number,
        const QString& previous_line,
        const QString& next_line)
    {
        GcodeCommand l, r;
        if(m_current_parser == nullptr)
        {
            throwParserNotSetException();
        }

        // Sets the internal state of the parser.
        m_current_parser->parseCommand(previous_line);
        l = m_current_parser->parseCommand(line, line_number);
        r = m_current_parser->parseCommand(next_line, line_number + 1);

        return QPair< GcodeCommand, GcodeCommand >(l, r);
    }

    void GcodeParser::selectParser(GcodeSyntax parserID)
    {
        if(m_current_parser_id == parserID)
        {
            return;
        }

        switch(parserID)
        {
            default:
                throwParserNotSetException();
                break;
        }

        m_current_parser->config();
    }

    void GcodeParser::freeParser(CommonParser *parser)
    {
        if(parser != nullptr)
        {
            delete parser;
        }

        parser = nullptr;
    }

    void GcodeParser::throwParserNotSetException()
    {
        QString exceptionString;
        QTextStream(&exceptionString)
            << "No parser selected to parse the GCode.";
        throw ParserNotSetException(exceptionString);
    }

    QString GcodeParser::getBlockCommentOpeningDelimiter()
    {
        return m_current_parser->getBlockCommentOpeningDelimiter();
    }

    Distance GcodeParser::getDistanceUnit()
    {
        return m_current_parser->getDistanceUnit();
    }
}  // namespace ORNL
#endif
