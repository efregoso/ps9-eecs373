//unload_box_v4.cpp:
// try to fill a single order, using both inspection  stations

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector2.h>

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>

#include<bin_inventory/bin_inventory.h>


const double COMPETITION_TIMEOUT = 500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

osrf_gear::Order g_order;
bool g_got_order = false;


// Function for converting a Model msg to a Part msg
void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}


// Listening for the Orders from ARIAC
void orderCallback(const osrf_gear::Order::ConstPtr& msg) {

    g_order = *msg;
    g_got_order = true;
    ROS_INFO("Received order %s with %i shipment%s", msg->order_id.c_str(), (int) msg->shipments.size(), msg->shipments.size() == 1 ? "" : "s");
    ROS_INFO_STREAM(g_order);
}


/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
	// Create a Service client for the correct service, i.e. '/ariac/start_competition'.
	ros::ServiceClient start_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	// If it's not already ready, wait for it to be ready.
	// Calling the Service using the client before the server is ready would fail.
	if (!start_client.exists()) {
		ROS_INFO("Waiting for the competition to be ready...");
		start_client.waitForExistence();
		ROS_INFO("Competition is now ready.");
	}
	ROS_INFO("Requesting competition start...");
	std_srvs::Trigger srv;  // Combination of the "request" and the "response".
	start_client.call(srv);  // Call the start Service.
	if (!srv.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
	} else {
		ROS_INFO("Competition started!");
	}
}


//Main method for starting the unload_box process. 
int main(int argc, char** argv) {

    // ROS set-ups:
    ros::init(argc, argv, "box_unloader"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    int ans;

    // Start the competition
    start_competition(nh);
    
    // Instantiate interfaces. 
    ROS_INFO("Instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("Instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);

    ROS_INFO("Instantiating a BoxInspector");
    BoxInspector2 boxInspector(&nh);

    ROS_INFO("Instantiating a binInventory object");
    BinInventory binInventory(&nh);
    inventory_msgs::Inventory current_inventory;

    ROS_INFO("Instantiating a drone client");
    ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl droneControl;

    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part, desired_part;
    geometry_msgs::PoseStamped box_pose_wrt_world; //camera sees box, coordinates are converted to world coords

    bool status;
    int nparts;

    // Subscribe to orders topic.
    ros::Subscriber sub = nh.subscribe("ariac/orders", 5, orderCallback);
    ROS_INFO("Waiting for order...");
    while (!g_got_order) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        if (!g_got_order) ROS_INFO("Waiting");

    }

    //For box inspector, need to define multiple vectors for args.  BoxInspector will identify parts and convert their coords to world frame.  In the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
    vector<osrf_gear::Model> desired_models_wrt_world;
    vector<osrf_gear::Model> satisfied_models_wrt_world;
    vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
    vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
    vector<osrf_gear::Model> missing_models_wrt_world;
    vector<osrf_gear::Model> orphan_models_wrt_world;
    vector<int> part_indices_missing;
    vector<int> part_indices_misplaced;
    vector<int> part_indices_precisely_placed;


    //Use conveyor action server for multi-tasking
    ROS_INFO("Getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1(); //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("Waiting for conveyor to advance a box to Q1...");
        }
    }

    //Update box pose,  if possible              
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("Box seen at: " << box_pose_wrt_world << endl);
    }
    else {
        ROS_WARN("No box seen at Q1 -- quitting.");
        exit(1);
    }
    
    //If survive to here, then box is at Q1 inspection station;.
    
    //Q1, Inspection 1: Compute desired  part poses w/rt world, given box location:
    boxInspector.compute_shipment_poses_wrt_world(g_order.shipments[0],box_pose_wrt_world,desired_models_wrt_world);
     
    //Q1, Inspection 1: Inspect the box and classify all observed parts
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("Q1, Inspection 1: Orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("Q1, Inspection 1: Num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("Q1, Inspection 1: Orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
   
    //Q1, Inspection 1: If a bad part is found, remove it. 
    if (boxInspector.get_bad_part_Q1(current_part)) {
        ROS_INFO("Q1, Inspection 1: Found bad part: ");
        ROS_INFO_STREAM(current_part<<endl);
        
        cout<<"Enter 1 to attempt to remove bad part: "; //poor-man's breakpoint
        cin>>ans;  
	//Q1, Inspection 1: Pick the part from the box and discard it.       
        status = robotBehaviorInterface.pick_part_from_box(current_part);
        status = robotBehaviorInterface.discard_grasped_part(current_part);
    }    

    //Q1, Inspection 1: After removing the bad part, re-inspect the box:
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("Q1, Reinspection 1: Orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("Q1, Reinspection 1: Num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("Q1, Reinspection 1: Orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }


    //Q1, Inspection 2: Move robot to grasp and discard each part.  Box inspector sees "model", defined in osrf_gear; convert this to our datatype "Part".  SHOULD do this for ALL orphaned parts
    nparts = orphan_models_wrt_world.size();
    while (nparts> 0) {
       model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

       //Q1, Inspection 2 - Use the robot as to grasp the bad part in the box and discard it. 
       status = robotBehaviorInterface.pick_part_from_box(current_part);
        status = robotBehaviorInterface.discard_grasped_part(current_part);    
        
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
       //this loop focuses on orphans, which includes bad parts
        nparts = orphan_models_wrt_world.size();
    }

    //SHOULD REPEAT FOR ALL THE PARTS IN THE BOX
    //ALSO, WATCH OUT FOR NO PARTS IN THE BOX--ABOVE WILL CRASH
    ROS_INFO("Q1, Inspection 2: Done removing orphans; attempt part relocations, as necessary");
        cout<<"Enter 1 to re-inspect: "; //poor-man's breakpoint
        cin>>ans;  
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
        nparts = orphan_models_wrt_world.size();        


    // Q1, Inspection 3: Find misplaced parts. 
    int nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
    bool go_on = false;
    ROS_INFO("Q1, Inspection 3: Found %d misplaced parts",nparts_misplaced);
    while (nparts_misplaced>0) {
        model_to_part(misplaced_models_actual_coords_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        int index_des_part = part_indices_misplaced[0];
        model_to_part(desired_models_wrt_world[index_des_part], desired_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        ROS_INFO("Q1, Inspection 3: Move part from: ");
        ROS_INFO_STREAM(current_part);
        ROS_INFO("Q1, Inspection 3: Move part to: ");
        ROS_INFO_STREAM(desired_part);
        cout<<"Enter 1 to reposition part"<<endl;
        cin>>ans;
       //Q1, Inspection 3: Use the robot action server to grasp part in the box:
        status = robotBehaviorInterface.pick_part_from_box(current_part); 
        
        //Q1, Inspection 3: Following fnc works ONLY if part is already grasped:
        status = robotBehaviorInterface.adjust_part_location_no_release(current_part,desired_part);
        status = robotBehaviorInterface.release_and_retract();
        
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
        nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
       
    }

	//Q1, Inspection 4: Populate missing parts in box:
	ROS_INFO("Q1, Inspection 4: Checking for parts missing in the box.");
    int n_missing_parts = part_indices_missing.size();
    while (n_missing_parts > 0) {
        int n_missing_part = part_indices_missing[0];

        std::string part_name(desired_models_wrt_world[n_missing_part].type);

        ROS_INFO_STREAM("Q1, Inspection 4: Looking for part " << part_name << endl);
        int partnum_in_inventory;
        bool part_in_inventory = true;
        inventory_msgs::Part pick_part, place_part;

	// Update the bin inventory object and find the part needing to be replaced. 
        binInventory.update();
        binInventory.get_inventory(current_inventory);
        part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
        if (!part_in_inventory) {
            ROS_WARN("Q1, Inspection 4: Could not find desired  part in inventory; giving up on process_part()");
            return false; //nothing more can be done     
        }
        ROS_INFO_STREAM("Q1, Inspection 4: Found part: " << pick_part << endl);
        //specify place part:
        model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_2);

        go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
        if (!go_on) {
            ROS_WARN("Q1, Inspection 4: Could not compute key pickup and place poses for this part source and destination");
        }


        ROS_INFO("Q1, Inspection 4: Attempting pick...");
        ROS_INFO("Q1, Inspection 4: Attempting to pick part");
        cout << "Enter 1:";
	//Q1, Inspection 4 - Pick up part from bin
        if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
            ROS_INFO("Q1, Inspection 4: Pick failed");
            go_on = false;
            return false;   
        }
	//Q1, Inspection 4 - Approach box
        ROS_INFO("Q1, Inspection 4: Moving to approach pose");
        if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
            ROS_WARN("Q1, Inspection 4: Could not move to approach pose");
            go_on = false;
            robotBehaviorInterface.discard_grasped_part(place_part);
        }
        //Q1, Inspection 4 - Place  part:
        ROS_INFO("Q1, Inspection 4: Attempting to place part");
        cout << "Wnter 1:";
        cin>>ans;
        if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
            ROS_INFO("Q1, Inspection 4: Placement failed");
            go_on = false;
            return false;
        }
	// Q1, Inspection 4: Release part into box and retract machine.
        status = robotBehaviorInterface.release_and_retract();

	// Q1, Inspection 4: Update inspection and replace more if necessary.
        boxInspector.update_inspection(desired_models_wrt_world,
                satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
                misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
                orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
                part_indices_precisely_placed);
        nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
        //Q1, Inspection 4: Populate missing parts in box:
        n_missing_parts = part_indices_missing.size();
    }
    ROS_INFO("Q1, Inspection 4: Done filling box; attempt part relocations, as necessary");


    //Advance the box further!
    conveyorInterface.move_box_Q1_to_Q2();

    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("Waiting for conveyor to advance a box to Q2...");
        }
    }

    //Update box pose, if possible      
    cout << "enter 1 to get box pose at Q2: ";
    cin>>ans;
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world, CAM2)) {
        ROS_INFO_STREAM("Q2: Box seen at: " << box_pose_wrt_world << endl);
    } else {
        ROS_WARN("No box seen at Q2 -- quitting.");
        exit(1);
    }


    // If survive to here, then box is at Q2 inspection station; 

    //Q2, Inspection 1: Compute desired  part poses w/rt world, given box location:
    boxInspector.compute_shipment_poses_wrt_world(g_order.shipments[0], box_pose_wrt_world, desired_models_wrt_world);

    //WIP: Q2, Inspection 1: Manipulate the objects in the box so that one of them is misplaced. 
    


    //Q2, Inspection 1: Inspect the box and classify all observed parts
    boxInspector.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, CAM2);
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("Q2, Inspection 1: Num orphaned parts seen in box = %d", nparts);
    for (int i = 0; i < nparts; i++) {
        ROS_INFO_STREAM("Q2, Inspection 1: Orphaned  parts: " << orphan_models_wrt_world[i] << endl);
    }

    if (boxInspector.get_bad_part_Q(current_part, CAM2)) {
        ROS_INFO("Q2, Inspection 1: Found bad part: ");
        ROS_INFO_STREAM(current_part << endl);

        cout << "Enter 1 to attempt to remove bad part: "; //poor-man's breakpoint
        cin>>ans;
	
       //Q2, Inspection 1 - Use the robot as to grasp the bad part in the box and discard it. 
        status = robotBehaviorInterface.pick_part_from_box(current_part);
        status = robotBehaviorInterface.discard_grasped_part(current_part);

    }

    //Q2, Inspection 2 - After removing the bad part, re-inspect the box:
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed, CAM2);
    ROS_INFO("Q2, Reinspection 1: Orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("Q2, Reinspection 1: Num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("Q2, Reinspection 1: Orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }


    //Q2, Inspection 2 - Move robot to grasp and discard each part.  Box inspector sees "model", defined in osrf_gear; convert this to our datatype "Part".  SHOULD do this for ALL orphaned parts
    nparts = orphan_models_wrt_world.size();
    while (nparts> 0) {
       model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

       //Q2, Inspection 2 - Use the robot as to grasp the bad part in the box and discard it. 
       status = robotBehaviorInterface.pick_part_from_box(current_part);
        status = robotBehaviorInterface.discard_grasped_part(current_part);    
        
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed, CAM2);
       //this loop focuses on orphans, which includes bad parts
        nparts = orphan_models_wrt_world.size();
    }

    //SHOULD REPEAT FOR ALL THE PARTS IN THE BOX
    //ALSO, WATCH OUT FOR NO PARTS IN THE BOX--ABOVE WILL CRASH
    ROS_INFO("Q2, Inspection 2: Done removing orphans; attempt part relocations, as necessary");
        cout<<"Enter 1 to re-inspect: "; //poor-man's breakpoint
        cin>>ans;  
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed, CAM2);
        nparts = orphan_models_wrt_world.size();        


    // Q2, Inspection 3: Find misplaced parts. 
    nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
    go_on = false;
    ROS_INFO("Q2, Inspection 3: Found %d misplaced parts",nparts_misplaced);
    while (nparts_misplaced>0) {
        model_to_part(misplaced_models_actual_coords_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        int index_des_part = part_indices_misplaced[0];
        model_to_part(desired_models_wrt_world[index_des_part], desired_part, inventory_msgs::Part::QUALITY_SENSOR_1);
        ROS_INFO("Q2, Inspection 3: Move part from: ");
        ROS_INFO_STREAM(current_part);
        ROS_INFO("Q2, Inspection 3: Move part to: ");
        ROS_INFO_STREAM(desired_part);
        cout<<"Enter 1 to reposition part"<<endl;
        cin>>ans;
       //Q1, Inspection 3: Use the robot action server to grasp part in the box:
        status = robotBehaviorInterface.pick_part_from_box(current_part); 
        
        //Q1, Inspection 3: Following fnc works ONLY if part is already grasped:
        status = robotBehaviorInterface.adjust_part_location_no_release(current_part,desired_part);
        status = robotBehaviorInterface.release_and_retract();
        
       boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed, CAM2);
        nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
       
    }

	//Q2, Inspection 4: Populate missing parts in box:
	ROS_INFO("Q2, Inspection 4: Checking for parts missing in the box.");
    n_missing_parts = part_indices_missing.size();
    while (n_missing_parts > 0) {
        int n_missing_part = part_indices_missing[0];

        std::string part_name(desired_models_wrt_world[n_missing_part].type);

        ROS_INFO_STREAM("Q2, Inspection 4: Looking for part " << part_name << endl);
        int partnum_in_inventory;
        bool part_in_inventory = true;
        inventory_msgs::Part pick_part, place_part;

	// Update the bin inventory object and find the part needing to be replaced. 
        binInventory.update();
        binInventory.get_inventory(current_inventory);
        part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
        if (!part_in_inventory) {
            ROS_WARN("Q2, Inspection 4: Could not find desired  part in inventory; giving up on process_part()");
            return false; //nothing more can be done     
        }
        ROS_INFO_STREAM("Q2, Inspection 4: Found part: " << pick_part << endl);
        //specify place part:
        model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_2);

        go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
        if (!go_on) {
            ROS_WARN("Q2, Inspection 4: Could not compute key pickup and place poses for this part source and destination");
        }


        ROS_INFO("Q2, Inspection 4: Attempting pick...");
        ROS_INFO("Q2, Inspection 4: Attempting to pick part");
        cout << "Enter 1:";
	//Q2, Inspection 4 - Pick up part from bin
        if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
            ROS_INFO("Q2, Inspection 4: Pick failed");
            go_on = false;
            return false;   
        }
	//Q2, Inspection 4 - Approach box
        ROS_INFO("Q2, Inspection 4: Moving to approach pose");
        if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
            ROS_WARN("Q2, Inspection 4: Could not move to approach pose");
            go_on = false;
            robotBehaviorInterface.discard_grasped_part(place_part);
        }
        //Q2, Inspection 4 - Place  part:
        ROS_INFO("Q2, Inspection 4: Attempting to place part");
        cout << "Enter 1:";
        cin>>ans;
        if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
            ROS_INFO("Q2, Inspection 4: Placement failed");
            go_on = false;
            return false;
        }
	// Q2, Inspection 4: Release part into box and retract machine.
        status = robotBehaviorInterface.release_and_retract();

	// Q2, Inspection 4: Update inspection and replace more if necessary.
        boxInspector.update_inspection(desired_models_wrt_world,
                satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
                misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
                orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
                part_indices_precisely_placed, CAM2);
        nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
        //Q2, Inspection 4: Populate missing parts in box:
        n_missing_parts = part_indices_missing.size();
    }
    ROS_INFO("Q2, Inspection 4: Done filling box; attempt part relocations, as necessary");

    

    ROS_INFO("Advancing box to loading dock for shipment");
    conveyorInterface.move_box_Q2_to_drone_depot();

    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("Waiting for conveyor to advance a box to loading dock...");
        }
    }
    ROS_INFO("Calling drone");
    //g_order.shipments[0].shipment_type;
    droneControl.request.shipment_type = g_order.shipments[0].shipment_type;
    ROS_INFO_STREAM("Shipment name: " << g_order.shipments[0].shipment_type << endl);

    droneControl.response.success = false;
    while (!droneControl.response.success) {
        drone_client.call(droneControl);
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("Finished.");
    return 0;
    //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
    // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
    // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}
