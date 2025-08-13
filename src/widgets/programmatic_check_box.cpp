#include "widgets/programmatic_check_box.h"

namespace ORNL {

ProgrammaticCheckBox::ProgrammaticCheckBox(QString str, QWidget* parent) : QCheckBox(str, parent) { setTristate(true); }

void ProgrammaticCheckBox::nextCheckState() {
    if (this->checkState() == Qt::Checked)
        setCheckState(Qt::Unchecked);
    else
        setCheckState(Qt::Checked);
}
} // namespace ORNL
