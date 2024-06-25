#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/msgs/marker.pb.h>
#include <ignition/msgs/marker_v.pb.h>
#include <ignition/math/Vector3.hh>
#include <ignition/common.hh>
#include <random>
#include <string>
#include <vector>
#include <chrono>


// includes for rendering plugin
// #include <ignition/gazebo/rendering/RenderTypes.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <ignition/msgs/material.pb.h>
#include <sdf/sdf.hh>
#include <ignition/msgs/visual.pb.h>


// includes for ROS side msgs and services
// #include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>

using namespace ignition;
using namespace gazebo;
using namespace std::literals::chrono_literals;


class UwbPlugin : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate
{   

private:
    // struct to store the marker data
    struct MarkerData
        {
            ignition::math::Vector3d startPoint;
            ignition::math::Vector3d endPoint;
            std::string intersectionName;
            bool hasIntersection;
            int markerId;
            std::string LOS_type = "";
        };

    public: void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf, EntityComponentManager &ecm, EventManager &eventMgr) override
    {
        this->model = Model(entity);
        this->worldEntity = ecm.EntityByComponents(components::World());

        // this->isWorldEntity = ecm.Component<components::World>(entity) != nullptr;
        // if (this->isWorldEntity)
        // {
        //     std::cout << "Configured for the world entity with ID: " << entity << std::endl;
        // }
        // else
        // {
        //     std::cout << "Configured for the model entity with ID: " << entity << std::endl;
        // }

        // reading the update rate from sdf
        this->updateRate = sdf->Get<double>("update_rate", 1.0).first;
        // calling the set update rate function to set the update rate in terms of seconds
        this->SetUpdateRate(this->updateRate);
        // reading the beacon prefix from sdf
        this->beaconPrefix = sdf->Get<std::string>("beacon_prefix", "uwb_anchor").first;
        // read the tag link name from the sdf
        this->tagLinkName = sdf->Get<std::string>("tag_link", "tag_link").first;
        // read the tag ID number from the sdf
        this->tagId = sdf->Get<int>("tag_id", 0).first;
        // read the Set_dataum boolean flag from the sdf
        this->Set_dataum = sdf->Get<bool>("Set_dataum", false).first;
        // read the dataum pose from the sdf if the Set_dataum flag is true
        if (this->Set_dataum){
            this->datumPose = sdf->Get<ignition::math::Pose3d>("dataum_pose", ignition::math::Pose3d(0, 0, 0, 0, 0, 0)).first;
        }
        // read the tag frame id string from the sdf, or make it the same as the tag_link name
        this->tagFrameId = sdf->Get<std::string>("frame_id", tagLinkName).first;



        // get all the parameters from the sdf file
        // this->nlosSoftWallWidth = sdf->Get<double>("nlosSoftWallWidth", 0.25).first;
        // this->tagZOffset = sdf->Get<double>("tag_z_offset", 0.0).first;
        // this->allBeaconsAreLOS = sdf->Get<bool>("all_los", false).first;
        // this->tagId = sdf->Get<int>("tag_id", 0).first;
        // this->maxDBDistance = sdf->Get<double>("maxDBDistance", 14.0).first;
        // this->stepDBDistance = sdf->Get<double>("stepDBDistance", 0.1).first;
        // this->useParentAsReference = sdf->Get<bool>("useParentAsReference", false).first;

        std::cout << "[UWB Plugin] Status: running." << std::endl;
        std::cout << "[UWB Plugin] Update rate: " << sdf->Get<double>("update_rate") << std::endl;
        std::cout << "[UWB Plugin] Beacon prefix: " << sdf->Get<std::string>("beacon_prefix") << std::endl;
        std::cout << "[UWB Plugin] Tag link: " << sdf->Get<std::string>("tag_link") << std::endl;
        std::cout << "[UWB Plugin] Tag ID: " << sdf->Get<int>("tag_id") << std::endl;
        std::cout << "[UWB Plugin] Set dataum: " << sdf->Get<bool>("Set_dataum") << std::endl;
        if (this->Set_dataum){
            std::cout << "[UWB Plugin] Dataum Pose: " << sdf->Get<ignition::math::Pose3d>("dataum_pose") << std::endl;
        }
        std::cout << "[UWB Plugin] Tag Frame ID: " << sdf->Get<std::string>("frame_id") << std::endl;

        // print the tag link and id which is being tracked
        // std::cout << "[UWB Plugin] Tag ID being Tracked: " << this->tagId << std::endl;
        // print the tag link name
        // std::cout << "[UWB Plugin] Tag Link being Tracked: " << this->model.Name(ecm) << std::endl;
        // print the model
        // std::cout << "[UWB Plugin] Model: " << this->model.Name(ecm) << std::endl;
        //  print the world entity
        // std::cout << "[UWB Plugin] World Entity: " << this->worldEntity << std::endl;
        // std::cout << "[UWB Plugin] World Entity Name: " << this->worldEntity.Name(ecm)<< std::endl;

        
        // Initialize Ignition Transport node and publishers
        // std::string topicRanging = "/uwb/toa/ranging";
        // std::cout << "[UWB Plugin] Ranging Publishing in " << topicRanging << std::endl;
        // this->UwbPub = this->node.Advertise<ignition::msgs::Double>(topicRanging);

        // std::string topicAnchors = "/uwb/toa/anchors";
        // std::cout << "[UWB Plugin] Anchors Position Publishing in " << topicAnchors << std::endl;
        // this->Anchors = this->node.Advertise<ignition::msgs::Marker_V>(topicAnchors);

        //! [connectToServerEvent]
        this->connection = eventMgr.Connect<gz::sim::events::PreRender>(
            std::bind(&UwbPlugin::PerformRenderingOperations, this));
        //! [connectToServerEvent]


        // initialise the ros2 node
        // Check if the ROS context is already initialized
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        this->rosNode = rclcpp::Node::make_shared("uwb_plugin");
        this->rosNode->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // msg for the markers, line of LOS
        this->markerPub = this->rosNode->create_publisher<visualization_msgs::msg::Marker>("LOS_marker", 10);
        // msg for the pose of the robot with covariance
        this->posePub = this->rosNode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
        // msg for publishing the static pose of the beacons
        // Create a publisher with transient local durability QoS
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.transient_local(); // Ensure the message is persisted
        this->marker_pub_beacon = this->rosNode->create_publisher<visualization_msgs::msg::Marker>("Beacon_marker", qos);


        // initialize gazebo transport node
        

        // Pulishing the markers
        // this->LOS_marker = this->node.Advertise<ignition::msgs::Marker>("/uwb/LOS_marker");

        // create a publisher for the Gazebo LOS markers
        // this->LOS_marker = this->node.Advertise<ignition::msgs::Marker>("/marker");


        // used to get the beacond dedtails for the first time
        this->is_pligin_initialized = false;
    }

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm) override
    {

        // if the marker operations flag is set to true, perform the marker operations
        if (this->performMarkerOperations_flag)
        {
            // print performaing marker operations
            // std::cout << "[UWB Plugin] Performing Marker Operations" << std::endl;
            // Clear all previous markers
            // ignition::msgs::Marker clearMarker;
            // clearMarker.set_ns("uwb");
            // clearMarker.set_action(ignition::msgs::Marker::DELETE_ALL);
            // this->node.Request("/marker", clearMarker);
            // above code was clearing all the markers, but we only need to clear the LOS markers.
            // as we are using other marekes as well now
            // for (const int id : 5)
            // {
            //     ignition::msgs::Marker clearMarker;
            //     clearMarker.set_ns("uwb");
            //     clearMarker.set_id(id);
            //     clearMarker.set_action(ignition::msgs::Marker::);
            //     this->node.Request("/marker", clearMarker);
            // }

            int LOS_count = 0;

            // create a simple for loop, with range equal to the size of beacon entities
            for (int i = 0; i < this->beaconEntities.size(); i++)
            {
                // initialise the loop variables
                auto markerData = markerDataList[i];
                const auto beaconEntity = this->beaconEntities[i];
                const auto beaconVisualEntity = this->beaconVisualEntities[i];
                std::string LOS_type = markerData.LOS_type;

                // check if the intersection name the beaconprefix in its name
                // createa a boolean variable for flagging
                bool isBeacon = markerData.intersectionName.find(this->beaconPrefix) != std::string::npos;
    
                if (markerData.hasIntersection && !isBeacon)
                {   
                    // Set line marker color to red and publish
                    SetLineMarker(markerData, "red");
                    // if the beacon has NLOS, then make it red
                    // since we dont need to always chnage the color of the beacon, we only need chnage if the color was previous green(has LOS)
                    auto material = _ecm.Component<components::Material>(beaconVisualEntity);
                    if (material && LOS_type != "NLOS")
                    {
                        // call the function to set the model color
                        SetModelColor(beaconVisualEntity, "red");                       
                    }
                    // it is important to do it at then end of the chekc, as it represents if the state has chnaged or not, and then we can chnage the color
                    // and then set he LOS type to NLOS
                    // set the LOS type to NLOS
                    markerDataList[i].LOS_type = "NLOS";
                    // call the function to set the line marker using ROS only for the LOS condition
                    SetLineMarker_ROS(markerData, "red");
                }
                else
                {
                    // Set marker color to red and publish
                    SetLineMarker(markerData, "green");
                    // if the beacon has LOS, then make it green
                    auto material = _ecm.Component<components::Material>(beaconVisualEntity);
                    if (material && LOS_type != "LOS")
                    {
                        // call the function to set the model color
                        SetModelColor(beaconVisualEntity, "green");
                    }
                    // it is important to do it at then end of the chekc, as it represents if the state has chnaged or not, and then we can chnage the color
                    // and then set he LOS type to NLOS
                    // set the LOS type to LOS
                    markerDataList[i].LOS_type = "LOS";
                    // call the function to set the line marker using ROS only for the LOS condition
                    SetLineMarker_ROS(markerData, "green");

                    // increase the LOS count
                    LOS_count++;
                }

            }

            // Call the function to publish the Pose data, based on the LOS count
            PublishPoseWithCovariance(LOS_count);

            // unset the flag
            this->performMarkerOperations_flag = false;
        }
    }

    public: void PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm) override
    {   

        // run once for the first time to get the tag entity and the beacon entities
        if (!this->is_pligin_initialized){
            this->initialize_plugin(ecm);
        }

        this->simTime = info.simTime;
        auto simTimeSec = std::chrono::duration_cast<std::chrono::seconds>(this->simTime);
        auto simTimeNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(this->simTime) - simTimeSec;
        std::chrono::steady_clock::time_point simTime_converted = std::chrono::steady_clock::time_point(simTimeSec + simTimeNsec);
        this->elapsed = simTime_converted - this->lastUpdateTime;
        // display the elapsed time
        // std::cout << "[UWB Plugin] Elapsed Time: " << this->elapsed.count() << " seconds" << std::endl;


        // run only if the elapsed time is greater than the update period
        if (elapsed >= this->updatePeriod)
        {
            this->lastUpdateTime = simTime_converted;


            // get the tag pose, wrt to the robot model baselink
            const auto tag_PoseComp = ecm.Component<components::Pose>(this->tagEntity);
            this->tagPose = tag_PoseComp->Data();
            // std::cout << "[UWB Plugin] Tag Pose: " << this->tagPose << std::endl;

            // find the world pose of the tag entity
            this->tag_worldPose = ignition::gazebo::worldPose(this->tagEntity, ecm);
            // std::cout << "[UWB Plugin] Tag World Pose: " << this->tag_worldPose << std::endl;

            // getting the updated pose of the tag entity
            this->tagParentPose = ecm.Component<components::Pose>(this->parentEntity)->Data();

            // set the flag to perform rendering operations
            this->performRenderingOperations_flag = true;

            // Update logic, ranging calculations, etc.
            // ...
            // this->sequence++;
            // display the sequence
            // std::cout << "[UWB Plugin] Sequence: " << this->sequence << std::endl;
        }
    }

    //! [performRenderingOperations]
    private: void PerformRenderingOperations()
    {   

        if (nullptr == this->scene)
        {
            this->FindScene();
        }

        if (nullptr == this->scene)
        {
            // print scene not found
            std::cout << "[UWB Plugin] Scene not found" << std::endl;
            return;
        }

        // if (this->simTime - this->lastUpdate < 1s)
        //     return;

        // print the scene name
        // std::cout << "[UWB Plugin] Scene Name: " << this->scene->Name() << std::endl;
        // create a ray query object from the scene
        this->rayQuery = this->scene->CreateRayQuery();


        if (this->performRenderingOperations_flag)
        {   
            // clear the marker data list
            this->markerDataList.clear();
            int markerId = 1;
            // do ray casting for each beacon entity position and the tag position(hence usnig the tag world pose)
            // use tagparentpose if you want the raycasting wrt to the parent entity of the tag entity(robot model)
            // is using tag world pose, then the raycasting is wrt to the world frame, and we will need the frames and tf to be correct to be able 
            // calcualte the robots base link pose from it
            for (const auto &beaconEntity : this->beaconEntities_pos)
            {
                auto anchorPose = beaconEntity;
                // make a world pose wiht offset variable called tag_worldPose_offset
                ignition::math::Vector3d tag_worldPose_offset = this->tag_worldPose.Pos() + ignition::math::Vector3d(0, 0, 0.05);
                // finding the direction vector from the tag to the beacon
                ignition::math::Vector3d dir = anchorPose.Pos() - tag_worldPose_offset;
                dir.Normalize();
                // this->rayQuery->SetOrigin(this->tag_worldPose.Pos() + ignition::math::Vector3d(0, 0, 0.1));
                this->rayQuery->SetOrigin(tag_worldPose_offset);
                // this->rayQuery->SetOrigin(this->tag_worldPose.Pos());
                this->rayQuery->SetDirection(dir);

                ignition::rendering::RayQueryResult result = this->rayQuery->ClosestPoint();
                if (result){
                    ignition::math::Vector3d intersectionPoint = result.point;
                    // get the intersection object ID 
                    Entity intersectionEntityID = result.objectId;
                    // use Id to get the visual name
                    const auto intersectionNameComp = this->scene->VisualById(intersectionEntityID)->Name();
                    // print the intersection name
                    // std::cout << "[UWB Plugin] Intersection Name: " << intersectionNameComp << std::endl;
                    markerDataList.push_back({tag_worldPose_offset, intersectionPoint, intersectionNameComp, true, markerId});
                    // markerDataList.push_back({this->tag_worldPose.Pos(), intersectionPoint, intersectionNameComp, true, markerId});
                }
                else{
                    // markerDataList.push_back({this->tag_worldPose.Pos(), anchorPose.Pos(), "", false, markerId});
                    markerDataList.push_back({tag_worldPose_offset, anchorPose.Pos(), "", false, markerId});
                }
                markerId++;
            }
            // reset the flag
            this->performRenderingOperations_flag = false;
            // set the flag to perform marker operations
            this->performMarkerOperations_flag = true;

            // print the flags
            // std::cout << "[UWB Plugin] Perform Rendering Operations Flag: " << this->performRenderingOperations_flag << std::endl;
        }
        
        // update the time
        this->lastUpdate = this->simTime;
    }

    //! [findScene]
    void FindScene()
    {
        auto loadedEngNames = gz::rendering::loadedEngines();
        if (loadedEngNames.empty())
        {
            igndbg << "No rendering engine is loaded yet" << std::endl;
            return;
        }

        // assume there is only one engine loaded
        auto engineName = loadedEngNames[0];
        if (loadedEngNames.size() > 1)
        {
            igndbg << "More than one engine is available. "
            << "Using engine [" << engineName << "]" << std::endl;
        }
        auto engine = gz::rendering::engine(engineName);
        if (!engine)
        {
            ignerr << "Internal error: failed to load engine [" << engineName
            << "]. Grid plugin won't work." << std::endl;
            return;
        }

        if (engine->SceneCount() == 0)
        {
            igndbg << "No scene has been created yet" << std::endl;
            return;
        }

        // Get first scene
        auto scenePtr = engine->SceneByIndex(0);
        if (nullptr == scenePtr)
        {
            ignerr << "Internal error: scene is null." << std::endl;
            return;
        }

        if (engine->SceneCount() > 1)
        {
            igndbg << "More than one scene is available. "
            << "Using scene [" << scene->Name() << "]" << std::endl;
        }

        if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
        {
            return;
        }

        this->scene = scenePtr;
    }

    void SetUpdateRate(double _rate)
    {
        if (_rate > 0.0)
        {
            // Convert the rate to seconds
            double periodInSeconds = 1.0 / _rate;
            // Assign the update period
            this->updatePeriod = std::chrono::duration<double>(periodInSeconds);
        }
        else
        {
            // Set a default update period
            this->updatePeriod = std::chrono::duration<double>(1.0);
        }
        // print the update period
        std::cout << "[UWB Plugin] Update Period: " << this->updatePeriod.count() << " seconds" << std::endl;
    }

    // this function is called by the postupdate on the first run.
    // This is necessary as the tag, beacon and other models are not loaded in the configure function
    void initialize_plugin(const EntityComponentManager &ecm){
        // get all the beacons in the world
        // Since we are only adding the plugin code to one of the beacon models
        // it loads immidiaely and the other beacons from the sdf are not loaded yet in the configure function
        // so we get the rest of the beacons in the post update function, in the first iteration
        
        std::vector<Entity> beacons;
        ecm.Each<components::Name, components::Pose>(
            [&](const Entity &_entity, const components::Name *_name, const components::Pose *_pose) -> bool
            {
                if (_name->Data().find(this->beaconPrefix) == 0)
                {
                    beacons.push_back(_entity);
                }
                return true;
            });
        // print the number of beacons
        std::cout << "[UWB Plugin] Number of Beacons: " << beacons.size() << std::endl;
        // print the beacon names and entity ids
        for (auto &beacon : beacons)
        {
            std::cout << "[UWB Plugin] Beacon Name: " << ecm.Component<components::Name>(beacon)->Data() << std::endl;
            std::cout << "[UWB Plugin] Beacon Entity ID: " << beacon << std::endl;
        }

        // find the tag entity by searching for the tag link name 
        // it can be a sub string in the name of the component
        // get the tag entity by searching for the tag link name
        ecm.Each<components::Name>(
            [&](const Entity &_entity, const components::Name *_name) -> bool
            {
                if (_name->Data().find(this->tagLinkName) != std::string::npos)
                {
                    this->tagEntity = _entity;
                    return false;
                }
                return true;
            });
            // print the pose of the tag entity, this is wrt to the parent entity if the entity is a child of a mode, 
            // in this case it would usually be as visual entity of the tag
            std::cout << "[UWB Plugin] Tag Entity ID: " << this->tagEntity << std::endl;
            std::cout << "[UWB Plugin] Tag Entity Name: " << ecm.Component<components::Name>(this->tagEntity)->Data() << std::endl;
            // print pose
            const auto tag_PoseComp = ecm.Component<components::Pose>(this->tagEntity);
            this->tagPose = tag_PoseComp->Data();
            std::cout << "[UWB Plugin] Tag Pose: " << this->tagPose << std::endl;
            // find the world pose of the tag entity
            this->tag_worldPose = ignition::gazebo::worldPose(this->tagEntity, ecm);
            std::cout << "[UWB Plugin] Tag World Pose: " << this->tag_worldPose << std::endl;


        
        // The tag can only be attached as a link in the urdf file.
        // when converting from urdf to sdf, the tag link is converted to only a visual component as the robot model
        // can only links, not sub links, everthing else is collapsed by the urdf to sdf conversion
        // so this code is to get the visual component of the tag link, then the parent entity of the visual component whcih is a link
        // then the parent entity of the link which is the model entity(the robot model)
        // get parententity of the tag link using ecm
        const auto parentComp = ecm.Component<components::ParentEntity>(this->tagEntity);
        if (!parentComp){
            ignerr << "Parent entity component not found for tag link!" << std::endl;
            return;
        }
        else{
            // first parent entity of the tag link
            auto temp_parentEntity = parentComp->Data();

            // Check if the parent entity is a model by looking for the Model component
            auto modelComp = ecm.Component<ignition::gazebo::components::Model>(temp_parentEntity);
            if (modelComp){
                // if parent is a model component we get the pose of this model
                std::cout << "[UWB Plugin] First Parent Entity of the Tag Link: " << temp_parentEntity << std::endl;
                const auto tag_ParentPoseComp = ecm.Component<components::Pose>(temp_parentEntity);
                this-> tagParentPose = tag_ParentPoseComp->Data();
                this->parentEntity = temp_parentEntity;
            }
            else{
                // we find the parent entity of the parent entity/ since in urdf conversion 
                // only one model is created for the whole robot
                // we get the parent entity of the parent entity
                const auto parent_parentComp = ecm.Component<components::ParentEntity>(temp_parentEntity);
                // second parent entity of the tag link
                auto temp_parent_parentEntity = parent_parentComp->Data();
                std::cout << "[UWB Plugin] Second Parent Entity of the Tag Link: " << temp_parent_parentEntity << std::endl;

                
                // get the pose of the second parent entity
                const auto tag_ParentPoseComp = ecm.Component<components::Pose>(parent_parentComp->Data());
                this->tagParentPose = tag_ParentPoseComp->Data();
                this->parentEntity = parent_parentComp->Data();
            }
        }


        // store all the beacon related data into vectors for faster acces later
        this->beaconEntities.clear();
        this->beaconEntities_pos.clear();
        this->beaconVisualEntities.clear();
        auto models = ecm.EntitiesByComponents(components::Model());
        for (const auto &modelEntity : models)
        {
            auto modelNameComp = ecm.Component<components::Name>(modelEntity);
            if (modelNameComp->Data().find(this->beaconPrefix) == 0)
            {
                // storing the beacon entities
                this->beaconEntities.push_back(modelEntity);

                // getting the pose of the beacon entities
                auto anchorPose = ecm.Component<components::Pose>(modelEntity)->Data();
                this->beaconEntities_pos.push_back(anchorPose);

                // getting the link associated with the beacon entity and then the visual component of the link entity
                // as of now this is the best way to access it, as we need to loop over the components returns,
                // hence the break statements, as we assume onle one link and visual component
                auto linkEntities = ecm.ChildrenByComponents(modelEntity, components::Link());
                for (const auto &linkEntity : linkEntities)
                {
                    // Get the visual component of the link entity
                    auto visualEntities = ecm.EntitiesByComponents(components::ParentEntity(linkEntity), components::Visual());
                    for (const auto &visualEntity : visualEntities)
                    {
                        // store the visual entities
                        this->beaconVisualEntities.push_back(visualEntity);
                        // print the visual entity ID
                        std::cout << "[UWB Plugin] Visual Entity ID: " << visualEntity << std::endl;
                        // prin the visual entity name
                        std::cout << "[UWB Plugin] Visual Entity Name: " << ecm.Component<components::Name>(visualEntity)->Data() << std::endl;
                        break;
                    }
                    break;
                }
            }
        }


        // clear the beacon entities pose vector
        // loop through all the beaconsentities and store the pose data
        for (const auto &beaconEntity : this->beaconEntities)
        {
            // store the pose of all the beacons
        }


        // set the gazebo origin usng marker 
        SetAxisMarkers(ignition::math::Pose3d::Zero,100);
           
        // set the axsis marker arows for the datum
        // read the dataum pose from the sdf if the Set_dataum flag is true
        if (this->Set_dataum){
            SetAxisMarkers(this->datumPose,200);
            // this->datumPos
            // print setting the axis markers
            std::cout << "[UWB Plugin] Setting the Axis Markers" << std::endl;
        }


        
        

        // publish the beacon markers static pose using the this->beaconEntities_pos vector
        PublishBeaconMarkers_ROS();
        
        // set the plugin initialized flag to true
        this->is_pligin_initialized = true;
    }

    // function to set the line markers
    // function to carry out the marker message operations
    public: void SetLineMarker(const UwbPlugin::MarkerData markerData, const std::string color)
    {
        ignition::msgs::Marker marker;
        marker.set_ns("uwb");
        marker.set_id(markerData.markerId);

        // use the marker data struct to print the marker ID and the intersection name
        std::cout << "[UWB Plugin] Marker ID: " << markerData.markerId << std::endl;
        std::cout << "[UWB Plugin] Intersection Name: " << markerData.intersectionName << std::endl;

        // no need to set the frane ID, as ther is not way yo bridge the marker message to ROS, not implmented by the team yet.
        // // marker.mutable_header()->set_data("frame_id", "world");  // Set the frame ID here
        // auto headerData = marker.mutable_header()->add_data();
        // headerData->set_key("frame_id");
        // headerData->add_value("world");

        marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
        marker.set_type(ignition::msgs::Marker::LINE_STRIP);
        ignition::msgs::Set(marker.mutable_pose(), ignition::math::Pose3d::Zero);
        ignition::msgs::Set(marker.add_point(), markerData.startPoint);
        ignition::msgs::Set(marker.add_point(), markerData.endPoint);


        if (color == "red")
        {
            ignition::msgs::Set(marker.mutable_material()->mutable_ambient(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(marker.mutable_material()->mutable_diffuse(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(marker.mutable_material()->mutable_specular(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(marker.mutable_material()->mutable_emissive(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
        }
        else
        {
            ignition::msgs::Set(marker.mutable_material()->mutable_ambient(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            ignition::msgs::Set(marker.mutable_material()->mutable_diffuse(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            ignition::msgs::Set(marker.mutable_material()->mutable_specular(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            ignition::msgs::Set(marker.mutable_material()->mutable_emissive(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
        }


        // you cant set the scale of lines using markers
        // marker.mutable_scale()->set_x(1);  // Line width
        // ignition::msgs::Set(marker.mutable_scale(),ignition::math::Vector3d(1.0, 0.0, 0.0));
        // gz::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(1.0, 1.0, 1.0));

        // send the marker message
        this->node.Request("/marker", marker);
        // this->LOS_marker.Publish(marker);
    }
   
    // function to set the model color using msgs, for the beacons based on the LOS
    public: void SetModelColor(int beaconVisualEntity, std::string color) {
        ignition::msgs::Visual req;
        req.set_id(beaconVisualEntity);

        if (color == "green")
        {
            ignition::msgs::Set(req.mutable_material()->mutable_ambient(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            ignition::msgs::Set(req.mutable_material()->mutable_diffuse(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            ignition::msgs::Set(req.mutable_material()->mutable_specular(), ignition::math::Color(0.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(req.mutable_material()->mutable_emissive(), ignition::math::Color(0.0, 0.0, 0.0, 1.0));
        }
        else if (color == "red")
        {
            ignition::msgs::Set(req.mutable_material()->mutable_ambient(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(req.mutable_material()->mutable_diffuse(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(req.mutable_material()->mutable_specular(), ignition::math::Color(0.0, 0.0, 0.0, 1.0));
            ignition::msgs::Set(req.mutable_material()->mutable_emissive(), ignition::math::Color(0.0, 0.0, 0.0, 1.0));
        }

        std::string service = "/world/default/visual_config";
        ignition::msgs::Boolean res;
        bool result;
        unsigned int timeout = 100;

        bool executed = this->node.Request(service, req, timeout, res, result);
        if (!executed)
        {
            std::cerr << "Failed to set visual config" << std::endl;
        }
    }

    // function to set the line visualization markers using ROS2
    public: void SetLineMarker_ROS(const UwbPlugin::MarkerData markerData, std::string color)
    {
        // Create and send the ROS 2 marker message
        visualization_msgs::msg::Marker rosMarker;
        rosMarker.header.frame_id = "world";
        rosMarker.header.stamp = rclcpp::Clock().now();
        rosMarker.ns = "uwb";
        rosMarker.id = markerData.markerId;
        rosMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        rosMarker.action = visualization_msgs::msg::Marker::MODIFY;

        rosMarker.pose.position.x = 0;
        rosMarker.pose.position.y = 0;
        rosMarker.pose.position.z = 0;
        rosMarker.pose.orientation.x = 0.0;
        rosMarker.pose.orientation.y = 0.0;
        rosMarker.pose.orientation.z = 0.0;
        rosMarker.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point p;
        p.x = markerData.startPoint.X();
        p.y = markerData.startPoint.Y();
        p.z = markerData.startPoint.Z();
        rosMarker.points.push_back(p);

        p.x = markerData.endPoint.X();
        p.y = markerData.endPoint.Y();
        p.z = markerData.endPoint.Z();
        rosMarker.points.push_back(p);

        if (color == "red")
        {   
            // we do not need the red lines to be visible, so we set the alpha to 0
            rosMarker.color.r = 1.0;
            rosMarker.color.g = 0.0;
            rosMarker.color.b = 0.0;
            rosMarker.color.a = 0.0;
        }
        else
        {
            rosMarker.color.r = 0.0;
            rosMarker.color.g = 1.0;
            rosMarker.color.b = 0.0;
            rosMarker.color.a = 0.5;
        }

        rosMarker.scale.x = 0.03; // Line width
        
        this->markerPub->publish(rosMarker);
    }

    // function to publish the pose of the robot with covariance, based on the number of  beacons in LOS
    public: void PublishPoseWithCovariance(int numBeaconsInLOS)
    {
        ignition::math::Pose3d currentPose;
        // Transform the pose to the new datum if setDatum is true, else use the pose of the tag as is
        if (this->Set_dataum)
        {
            currentPose = this->datumPose.Inverse() * tag_worldPose;
        }
        else
        {
            currentPose = this->tag_worldPose;
        }



         // Create and send the ROS 2 pose with covariance message
        geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.frame_id = this->tagFrameId;
        poseMsg.header.stamp = this->rosNode->get_clock()->now();

        // Set the pose, this->tagParentPose will have the updates pose as it is being updated in the postupdate
        poseMsg.pose.pose.position.x = currentPose.Pos().X();
        poseMsg.pose.pose.position.y = currentPose.Pos().Y();
        poseMsg.pose.pose.position.z = currentPose.Pos().Z();
        poseMsg.pose.pose.orientation.x = currentPose.Rot().X();
        poseMsg.pose.pose.orientation.y = currentPose.Rot().Y();
        poseMsg.pose.pose.orientation.z = currentPose.Rot().Z();
        poseMsg.pose.pose.orientation.w = currentPose.Rot().W();

        // Adjust covariance and noise based on the number of beacons in LOS
        double covarianceValue;
        double noiseFactor;

        if (numBeaconsInLOS > 3)
        {
            covarianceValue = 0.01; // High accuracy
            noiseFactor = 0.01;
        }
        else if (numBeaconsInLOS == 3)
        {
            covarianceValue = 0.1; // Moderate accuracy
            noiseFactor = 0.1;
        }
        else if (numBeaconsInLOS == 2)
        {
            covarianceValue = 0.5; // Low accuracy
            noiseFactor = 0.5;
        }
        else
        {
            covarianceValue = 1.0; // Very low accuracy
            noiseFactor = 1.0;
        }

        // Set the covariance
        for (int i = 0; i < 36; ++i)
        {
            poseMsg.pose.covariance[i] = 0.0;
        }
        poseMsg.pose.covariance[0] = covarianceValue;
        poseMsg.pose.covariance[7] = covarianceValue;
        poseMsg.pose.covariance[14] = covarianceValue;
        poseMsg.pose.covariance[21] = covarianceValue;
        poseMsg.pose.covariance[28] = covarianceValue;
        poseMsg.pose.covariance[35] = covarianceValue;

        // Add noise to the pose based on noise factor
        poseMsg.pose.pose.position.x += noiseFactor * ((rand() % 100) / 100.0 - 0.5);
        poseMsg.pose.pose.position.y += noiseFactor * ((rand() % 100) / 100.0 - 0.5);
        poseMsg.pose.pose.position.z += noiseFactor * ((rand() % 100) / 100.0 - 0.5);

        // Publish the message
        this->posePub->publish(poseMsg);
    }

    // function to publish the static pose of the beacons
    void PublishBeaconMarkers_ROS()
    {
        int id = 0;
        for (const auto &pose : this->beaconEntities_pos)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->rosNode->get_clock()->now();
            marker.ns = "uwb";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = pose.Pos().X();
            marker.pose.position.y = pose.Pos().Y();
            marker.pose.position.z = pose.Pos().Z();
            marker.pose.orientation.x = pose.Rot().X();
            marker.pose.orientation.y = pose.Rot().Y();
            marker.pose.orientation.z = pose.Rot().Z();
            marker.pose.orientation.w = pose.Rot().W();

            // below values are from the beaocns blender file
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;

            // set the lifetime of the marker, 0 measn forever
            marker.lifetime = rclcpp::Duration(0, 0);

            marker_pub_beacon->publish(marker);
        }
    }

   public: void SetAxisMarkers(const ignition::math::Pose3d &pose, const int baseMarkerId)
    {
        // Extract position and orientation from the pose argument
        ignition::math::Vector3d position = pose.Pos();
        ignition::math::Quaterniond orientation = pose.Rot();

        // Print the pose of the datum
        std::cout << "[UWB Plugin] Datum Pose: " << pose << std::endl;
        // print the position and orientation
        std::cout << "[UWB Plugin] Position: " << position << std::endl;
        std::cout << "[UWB Plugin] Orientation: " << orientation << std::endl;

        // X-axis marker
        ignition::msgs::Marker xMarker;
        xMarker.set_ns("uwb");
        xMarker.set_id(baseMarkerId);
        xMarker.set_action(ignition::msgs::Marker::ADD_MODIFY);
        xMarker.set_type(ignition::msgs::Marker::CYLINDER);

        // Set the position and orientation of the cylinder
        ignition::math::Pose3d xPose(position + orientation.RotateVector(ignition::math::Vector3d(0.5, 0, 0)), orientation * ignition::math::Quaterniond(0, 0, 0));
        // ignition::math::Pose3d xPose(position + orientation.RotateVector(ignition::math::Vector3d(0.5, 0, 0)), orientation * ignition::math::Quaterniond(0, IGN_DTOR(90), 0));
        ignition::msgs::Set(xMarker.mutable_pose(), xPose);
        // print the xpose
        std::cout << "[UWB Plugin] X Pose: " << xPose << std::endl;

        // Set the scale of the cylinder (height and radius)
        xMarker.mutable_scale()->set_x(1.0);  // Height
        xMarker.mutable_scale()->set_y(0.1); // Radius
        xMarker.mutable_scale()->set_z(0.1); // Radius

        ignition::msgs::Set(xMarker.mutable_material()->mutable_ambient(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));
        ignition::msgs::Set(xMarker.mutable_material()->mutable_diffuse(), ignition::math::Color(1.0, 0.0, 0.0, 1.0));

        // Set the marker to be persistent
        xMarker.mutable_lifetime()->set_sec(0);
        xMarker.mutable_lifetime()->set_nsec(0);

        this->node.Request("/marker", xMarker);

        // Y-axis marker
        ignition::msgs::Marker yMarker;
        yMarker.set_ns("uwb");
        yMarker.set_id(baseMarkerId + 1);
        yMarker.set_action(ignition::msgs::Marker::ADD_MODIFY);
        yMarker.set_type(ignition::msgs::Marker::CYLINDER);

        // Set the position and orientation of the cylinder
        ignition::math::Pose3d yPose(position + orientation.RotateVector(ignition::math::Vector3d(0, 0.5, 0)), orientation * ignition::math::Quaterniond(0, 0, IGN_DTOR(90)));
        ignition::msgs::Set(yMarker.mutable_pose(), yPose);
        // print the y pose
        std::cout << "[UWB Plugin] Y Pose: " << yPose << std::endl;

        // Set the scale of the cylinder (height and radius)
        yMarker.mutable_scale()->set_x(1.0);  // Height
        yMarker.mutable_scale()->set_y(0.1); // Radius
        yMarker.mutable_scale()->set_z(0.1); // Radius

        ignition::msgs::Set(yMarker.mutable_material()->mutable_ambient(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
        ignition::msgs::Set(yMarker.mutable_material()->mutable_diffuse(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));

        // Set the marker to be persistent
        yMarker.mutable_lifetime()->set_sec(0);
        yMarker.mutable_lifetime()->set_nsec(0);

        this->node.Request("/marker", yMarker);

        // Z-axis marker
        ignition::msgs::Marker zMarker;
        zMarker.set_ns("uwb");
        zMarker.set_id(baseMarkerId + 2);
        zMarker.set_action(ignition::msgs::Marker::ADD_MODIFY);
        zMarker.set_type(ignition::msgs::Marker::CYLINDER);

        // Set the position and orientation of the cylinder
        ignition::math::Pose3d zPose(position + orientation.RotateVector(ignition::math::Vector3d(0, 0, 0.5)), orientation * ignition::math::Quaterniond(0, IGN_DTOR(90), 0));
        ignition::msgs::Set(zMarker.mutable_pose(), zPose);
        // print the z pose
        std::cout << "[UWB Plugin] Z Pose: " << zPose << std::endl;

        // Set the scale of the cylinder (height and radius)
        zMarker.mutable_scale()->set_x(1.0);  // Height
        zMarker.mutable_scale()->set_y(0.1); // Radius
        zMarker.mutable_scale()->set_z(0.1); // Radius

        ignition::msgs::Set(zMarker.mutable_material()->mutable_ambient(), ignition::math::Color(0.0, 0.0, 1.0, 1.0));
        ignition::msgs::Set(zMarker.mutable_material()->mutable_diffuse(), ignition::math::Color(0.0, 0.0, 1.0, 1.0));

        // Set the marker to be persistent
        zMarker.mutable_lifetime()->set_sec(0);
        zMarker.mutable_lifetime()->set_nsec(0);

        this->node.Request("/marker", zMarker);
    }



    private:
        Model model;
        Entity worldEntity;
        transport::Node node;
        // pubishers for the markers for LOS
        transport::Node::Publisher LOS_marker;

        // working on this
        double updateRate;
        std::string beaconPrefix;
        std::string tagLinkName;
        Entity tagEntity;
        int tagId;
        // flag to set a differnt reference frame for the UWB plugin
        bool Set_dataum;
        // pose of the datum
        ignition::math::Pose3d datumPose;
        // string for tag frame ID
        std::string tagFrameId;
 

        // boolean to check if the plugin is initialized.
        bool is_pligin_initialized;

        // for rendering related tasks
        ignition::rendering::v6::ScenePtr scene;
        ignition::rendering::RayQueryPtr rayQuery;
        ignition::rendering::v6::VisualPtr arrow;
        gz::common::ConnectionPtr connection{nullptr};
        // boolean to flag if rendering operations needs to be performed
        bool performRenderingOperations_flag{false};
        // boolean flag for performaing marker operations
        bool performMarkerOperations_flag{false};
        
        std::vector<MarkerData> markerDataList;



        ignition::math::Pose3d tagPose;
        ignition::math::Pose3d tag_worldPose;
        ignition::math::Pose3d tagParentPose;
        // make parent entity global
        Entity parentEntity;
        // vecotr for storing all the beacon entities
        std::vector<Entity> beaconEntities;
        // vector for storing pose of all beacon entities
        std::vector<ignition::math::Pose3d> beaconEntities_pos;
        // vector for storing the visual entities
        std::vector<Entity> beaconVisualEntities;
        

        unsigned char sequence;
        // double nlosSoftWallWidth;
        // double tagZOffset;
        // double maxDBDistance;
        // double stepDBDistance;
        // bool allBeaconsAreLOS;
        // bool isWorldEntity;
        // bool useParentAsReference;
        std::default_random_engine random_generator;
        std::chrono::duration<double> updatePeriod;
        std::chrono::steady_clock::time_point lastUpdateTime;
        std::chrono::duration<double> elapsed;
        std::chrono::_V2::steady_clock::duration simTime;
        std::chrono::steady_clock::duration lastUpdate{0};

        // ros node
        rclcpp::Node::SharedPtr rosNode;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
        // create a msg for pose of the robot with covariance
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub;
        // create a publisher for the static pose of the beacons
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_beacon;



        // // section for all the markers being pulished (dont think this will work as not implementd for bridging)
        // transport::Node::Publisher LOS_marker;
};

// Register this plugin with the simulator
IGNITION_ADD_PLUGIN(UwbPlugin, System, UwbPlugin::ISystemConfigure, UwbPlugin::ISystemPreUpdate, UwbPlugin::ISystemPostUpdate)