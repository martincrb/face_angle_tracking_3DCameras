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

