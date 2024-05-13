#include "Lidar.h"
#include "utils.h"


Lidar::Lidar(): clusters {}, cones {}, op_result {},
                channel {createSerialPortChannel("/dev/ttyUSB0", 115200)},
                lidar {*createLidarDriver()}
{
    lidar->connect(*channel);

    //lidar->setMotorSpeed(200);
}

Lidar::~Lidar()
{
    lidar->stop();
    delete lidar;
}

void Lidar::start()
{
    // start scan...
    lidar->startScan(false,true);
}

void Lidar::stop()
{
    lidar->stop();
}


void Lidar::update()
{
    clusters.clear();
    cones.clear();

    sl_lidar_response_measurement_node_hq_t  nodes[8192];
    size_t   count = _countof(nodes);

	// get the lidar data
    op_result = lidar->grabScanDataHq(nodes, count);

    // Check if result is invalid
    if ( !SL_IS_OK(op_result) )
    {
        return;
    }

    lidar->ascendScanData(nodes, count);

    // initialize previous distance to something very big (100 meters)
    float prev_distance = 100000.f;

    // loop  through all the points
    for (int pos = 0; pos < (int)count ; ++pos) {
        // converte to milimeters and radians
        float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f * M_PI / 180.f;
        float distance = nodes[pos].dist_mm_q2/4.0f;
        int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

        switch(quality){
        // not ok
        case 0:
            break;
        // ok
        case 47:
            //std::cout << "Angle: " << angle << "\tdistance " << distance << std::endl;
            // check if this should be the start of a new object
            if (abs( distance - prev_distance) > 200.f) {
                Cluster cluster {};
                clusters.push_back(cluster);
            }

            Point point;
            point.x = distance * cos(angle);
            point.y = distance * sin(angle);
            clusters.back().push_back(point);

            // uppdate prev_distance
            prev_distance = distance;
            break;
        }
    }

    // Concatenate first and last cluster if they are actually the same (circles)
    Point p1 {clusters.front().front()};
    Point p2 {clusters.back().back()};

    if ( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) < 200.f*200.f )
    {
        clusters.front().insert(clusters.front().begin(),
                                clusters.back().begin(),
                                clusters.back().end());
        clusters.pop_back();
    }

    // Remove small clusters
    filter();
    
    findCones();
}

std::vector<Cluster> Lidar::getPoints()
{
    return clusters;
}


std::vector<Cone> Lidar::getCones()
{
    return cones;
}


void Lidar::filter()
{
	// Remove clusters which have few datapoints
    
	// indicies of small clusters
	std::vector<int> small_clusters {};
	
	// Find small clusters
	for (int i {0}; i < clusters.size(); i++)
	{
 		if (clusters.at(i).size() < 2)
    	{
			small_clusters.push_back(i);
	    }
   	}
	// Remove small clusters
    for (int i { static_cast<int>(small_clusters.size()) - 1 }; i >= 0; i--)
    {
    	clusters.erase(clusters.begin() + small_clusters.at(i));
    }
    return;
}

void Lidar::findCones()
{
	// Create cones from clusters filtering out too large objects
	for (auto cluster : clusters)
	{
        // Calculate midpoint and radius from first and last point in cluster
        float x1 {cluster.front().x};
        float y1 {cluster.front().y};
        float x2 {cluster.back().x};
        float y2 {cluster.back().y};
        float r { sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) ) / 2 };
        
        // To large to be a cone
		if ( r > ( 190 + 110 ) / 2 )
		{
			continue;
		}
        // Large cone
        else if ( r > ( 120 + 40 ) / 2 )
        {
            r = 190.f / 2;
        }
        // Small cone
        else
        {
            r = 120.f / 2;
        }

		Cone cone;
		cone.x = (x1 + x2)/2;
		cone.y = (y1 + y2)/2;
        cone.r = r;
		cones.push_back(cone);
	}

	return;
}

