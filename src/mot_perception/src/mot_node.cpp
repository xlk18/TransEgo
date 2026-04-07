#include "../include/mot_node_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mot_node"); 
    ros::NodeHandle nh;                

    MOTNode node(nh); 

    ros::spin(); 
    return 0;    
}
