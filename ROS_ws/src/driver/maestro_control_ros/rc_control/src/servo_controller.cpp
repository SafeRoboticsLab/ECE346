# include <servo_controller.h>


ServoController::ServoController(ros::NodeHandle node, ros::NodeHandle private_nh):
    _nh(node), _pvt_nh(private_nh), _running(false), _cmd_vel_timeout(0.5)
{
    // set dynamic reconfig
    _f = boost::bind(&ServoController::_dynParamCallback, this, _1, _2);
    _dyn_reconfig_server.setCallback(_f);

    // obtain parameters from the launch file
    _ReadParameter();

    // Find if the desired controller is connected
    _device_idx = _SearchController();
    if(_device_idx<0)
    {
        ROS_ERROR("The controller is not intialized after 30 secs!");
        return;
    } 
    _running = true; 

    // get a copy of the controller setting
    Maestro::Device::ChannelSettings steering_ch_setting = _device_list[_device_idx ].getChannelSettings(_steering_ch);
    _steering_min = steering_ch_setting.minimum;
    _steering_max = steering_ch_setting.maximum;

    Maestro::Device::ChannelSettings throttle_ch_setting = _device_list[_device_idx ].getChannelSettings(_throttle_ch);
    _throttle_min = throttle_ch_setting.minimum;
    _throttle_max = throttle_ch_setting.maximum;

    /* 
    Initialize the controller
    */
    _dynParamUpdate();
    // Allow full speed and acceleration
    _device_list[_device_idx].setSpeed(_steering_ch, 0);
    _device_list[_device_idx].setSpeed(_throttle_ch, 0);
    _device_list[_device_idx].setAcceleration(_steering_ch, 0);
    _device_list[_device_idx].setAcceleration(_throttle_ch, 0);
    natural();

     // set up ros subscriber
    _sub_command = _nh.subscribe(_sub_topic, 1 , &ServoController::_subCallback, this);
    ROS_INFO_STREAM("Controller subscribe to topic: "<<_sub_topic);

}

ServoController::~ServoController()
{
    natural();
    ROS_DEBUG("Exit");
}

int ServoController::_SearchController()
{
    int index = -1;
    bool first_error = true;
    // scan all connected device
    _device_list = Maestro::Device::getConnectedDevices();
    for(int i =0; i<10; i++){
        if(_device_list.size() == 0){
            if(first_error)
            {
                ROS_WARN("No controller device found! Will retry every second");
                first_error = false;
            }
            sleep(1.0);
        }else //if(_device_list.size() == 1)
        {
            index = 0;
            ROS_INFO("Connected to the device: %s",(_device_list[index].getName()).c_str());
            break;
        }
    }

    /* Currently ignore that we have multiple servo connected    
    else{
        for(auto device = _device_list.begin(); device != _device_list.end(); device++)
            ROS_INFO("Found device: %s",(device->getName()).c_str());
    }
    */
   return index;
}

void ServoController::_ReadParameter()
{
    ROS_INFO("*** PARAMETER SETTINGS ***");

    _pvt_nh.param<std::string>("ControllerTopic", _sub_topic, "control");

    // Channel index for throttle 
    _pvt_nh.param("SteeringChannel", _steering_ch, -1);
    if(_steering_ch<0){
        ROS_ERROR_STREAM("Invalid steering channel "<<_steering_ch);
        _running = false;
    }else
        ROS_INFO_STREAM("Set channel "<<_steering_ch<<" to steering.");

    // Channel index for throttle 
    _pvt_nh.param("ThrottleChannel", _throttle_ch, -1);

    if(_throttle_ch<0){
        ROS_ERROR_STREAM("Invalid throttle channel "<<_throttle_ch);
        _running = false;
    }else
        ROS_INFO_STREAM("Set channel "<<_throttle_ch<<" to throttle.");

    if(_throttle_ch == _steering_ch){
        ROS_ERROR("Steering and Throttle are set to the same channel. Check your setting in the launch file.");
        _running = false;
    }
}

void ServoController::_dynParamCallback(rc_control::calibrationConfig &config, uint32_t level)
{
    DynParam dynamic_params;
    dynamic_params.steering_C = config.steering_C;
    dynamic_params.steering_L = config.steering_L;
    dynamic_params.steering_H = config.steering_H;

    dynamic_params.throttle_N = config.throttle_N;
    dynamic_params.throttle_L = config.throttle_L;
    dynamic_params.throttle_H = config.throttle_H;

    dynamic_params.inverse_steer = config.inverse_steer;
    dynamic_params.inverse_throttle = config.inverse_throttle;

    _dynParam.writeFromNonRT(dynamic_params);
}

void ServoController::_dynParamUpdate()
{
    const DynParam dynamic_params = *(_dynParam.readFromRT());
    _steering_C = dynamic_params.steering_C;
    _steering_L = dynamic_params.steering_L;
    _steering_H = dynamic_params.steering_H;
    

    _steering_L = std::min(_steering_C, _steering_L);
    _steering_H = std::max(_steering_C, _steering_H);

    _throttle_N = dynamic_params.throttle_N;
    _throttle_L = dynamic_params.throttle_L;
    _throttle_H = dynamic_params.throttle_H;

    _throttle_L = std::min(_throttle_N, _throttle_L);
    _throttle_H = std::max(_throttle_N, _throttle_H);

    _inverse_steer = dynamic_params.inverse_steer;
    _inverse_throttle = dynamic_params.inverse_throttle;
}

void ServoController::_subCallback(const rc_control_msgs::RCControl& msg)
{
    if(_running){
        if (_sub_command.getNumPublishers()>1)
        {
            ROS_ERROR_STREAM("Detected " << _sub_command.getNumPublishers()<< " publishers. Only 1 publisher is allowed. Going to brake.");
            brake();
            return;
        }
        
        _command_struct.steer = msg.steer;
        _command_struct.throttle = msg.throttle;
        _command_struct.reverse = msg.reverse;
        _command_struct.stamp = ros::Time::now();
        _command.writeFromNonRT(_command_struct);
    
    }
}

int ServoController::_convertTargetThrottle(double percentage)
{
    /* 
    convert the percentage input (-100% - 100%) 
    to the intergral input of pluse width (in unit of 0.25us)
    */
   
    if(_inverse_throttle)
        percentage = -percentage;
    
    double input_pwm = (percentage*400.0+_throttle_N); 

    // saturated to the limit
    input_pwm = std::max(_throttle_L, std::min(input_pwm, _throttle_H));

    // scaled to the unit of 0.25us
    int input = int(input_pwm*4.0);

    return std::max(_throttle_min, std::min(input, _throttle_max));
}

int ServoController::_convertTargetSteering(double percentage)
{
    /* 
    convert the percentage input (-100% - 100%) 
    to the intergral input of pluse width (in unit of 0.25us)
    */
    if (_inverse_steer)
        percentage = -percentage;

    double input_pwm = (percentage*400.0+_steering_C); 

    // saturated to the limit
    input_pwm = std::max(_steering_L, std::min(input_pwm, _steering_H));

    // scaled to the unit of 0.25us
    int input = int(input_pwm*4.0);

    return std::max(_steering_min, std::min(input, _steering_max));
}

void ServoController::brake()
{
    if(_running){
        if(_reverse)
        {
            _device_list[_device_idx].setTarget(_steering_ch, _convertTargetSteering(0));
            _device_list[_device_idx].setTarget(_throttle_ch, _convertTargetThrottle(1));   
        }else
        {   
            _device_list[_device_idx].setTarget(_steering_ch, _convertTargetSteering(0));
            _device_list[_device_idx].setTarget(_throttle_ch, _convertTargetThrottle(-1));   
        }
    }
}

void ServoController::natural()
{
    if(_running){
        _device_list[_device_idx].setTarget(_steering_ch, _convertTargetSteering(0));
        _device_list[_device_idx].setTarget(_throttle_ch, _convertTargetThrottle(0));
    }
}


void ServoController::update(const ros::Time& time, const ros::Duration& period)
{
    if(_running){
        _dynParamUpdate();
        Commands curr_cmd = *(_command.readFromRT());
        // ROS_INFO_STREAM("Receive control - Steer: "<<curr_cmd.steer
        //                         <<" Throttle: "<<curr_cmd.throttle
        //                         <<" Time: "<<curr_cmd.stamp);
        const double dt = (time - curr_cmd.stamp).toSec();
        
        if (dt > _cmd_vel_timeout){
            natural();
            return;
        }
        
        _device_list[_device_idx].setTarget(_steering_ch, _convertTargetSteering(curr_cmd.steer));
        _device_list[_device_idx].setTarget(_throttle_ch, _convertTargetThrottle(curr_cmd.throttle));
    }
}