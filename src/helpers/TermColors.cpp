#include <helpers/TermColor.h>

namespace helpers
{

    const std::string TermColor::RESET()
    {
        std::stringstream buffer;
        buffer << "\033[" << CTRL_RESET << "m";
        return buffer.str();
    }

    const std::string TermColor::compose(const int modifier, const int color, bool invert)
    {

        std::stringstream buffer;
        if (!invert)
            buffer << "\033[" << modifier << ";" << color << "m";
        else
            buffer << "\033[" << CTRL_INVERSE << ";" << modifier << ";" << color << "m";
        return buffer.str();
    }

    const std::string TermColor::compose(const int color, bool invert)
    {
        std::stringstream buffer;
        if (!invert)
            buffer << "\033[" << color << "m";
        else
            buffer << "\033[" << CTRL_INVERSE << ";" << color << "m";
        return buffer.str();
    }

    const std::string TermColor::RED() { return compose(TermColor::BG_RED); }
    const std::string TermColor::GREEN() { return compose(TermColor::BG_GREEN); }
    const std::string TermColor::YELLOW() { return compose(TermColor::BG_YELLOW); }
    const std::string TermColor::BLUE() { return compose(TermColor::BG_BLUE); }
    const std::string TermColor::MAGENTA() { return compose(TermColor::BG_MAGENTA); }
    const std::string TermColor::CYAN() { return compose(TermColor::BG_CYAN); }
    const std::string TermColor::WHITE() { return compose(TermColor::BG_WHITE); }

    const std::string TermColor::iRED() { return compose(TermColor::BG_RED, true); }
    const std::string TermColor::iGREEN() { return compose(TermColor::BG_GREEN, true); }
    const std::string TermColor::iYELLOW() { return compose(TermColor::BG_YELLOW, true); }
    const std::string TermColor::iBLUE() { return compose(TermColor::BG_BLUE, true); }
    const std::string TermColor::iMAGENTA() { return compose(TermColor::BG_MAGENTA, true); }
    const std::string TermColor::iCYAN() { return compose(TermColor::BG_CYAN, true); }
    const std::string TermColor::iWHITE() { return compose(TermColor::BG_WHITE, true); }

} // namespace helpers