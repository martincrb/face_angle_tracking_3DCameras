#include "TrackingAlgorithm.h"


TrackingAlgorithm::TrackingAlgorithm()
{
}


TrackingAlgorithm::~TrackingAlgorithm()
{
}

void TrackingAlgorithm::setParameters(QMap<QString, QString> p) {
	params = p;
}
QMap<QString, QString> TrackingAlgorithm::getParameters() {
	return params;
}

QString TrackingAlgorithm::getParameter(const QString key) {
	return params[key];
}
