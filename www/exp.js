//Send task to Gabe
function sendTask(task){
  var sendTaskClient = new ROSLIB.Service({
    ros : ros,
    name : '/task',
    serviceType : 'tablet_interface/Task'
  });

var request = new ROSLIB.ServiceRequest({
    name : task
  });
  // Finally, we call the /add_two_ints service and get back the results in the callback. The result
  // is a ROSLIB.ServiceResponse object.
  sendTaskClient.callService(request, function(result) {
    console.log('Result for service call on ' + sendTaskClient.name + ': ' + result.success);
    //On success
    if (result.succes == true){
      document.getElementById('failure').style.display = 'none';
      document.getElementById('success').style.display = 'inline';
    }
    //On Failure
    if (result.success == false){
      document.getElementById('failure').style.display = 'inline';
      document.getElementById('success').style.display = 'none';
    }

  });
}


//Access Battery Information
var globalTime = new Date();
var timeHolder = globalTime.getTime();

var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/sensor_state',
  messageType : 'turtlebot3_msgs/SensorState'
});

listener.subscribe(function(message) {
  var testTime = new Date();
  var testTime2 = testTime.getTime();
  var batPercent = "";

  if(timeHolder + 1000 < testTime2)
  {
    globalTime = new Date();
    timeHolder = globalTime.getTime();
    document.getElementById('battery').innerHTML = message.battery.toFixed(2) + "V";

    if(message.battery > 4.2 * 3){
      batPercent = "100%";
    }
    if(message.battery < 4.13 * 3){
      batPercent = "90%";
    }
    if(message.battery < 4.06 * 3){
      batPercent = "80%";
    }
    if(message.battery < 3.99 * 3){
      batPercent = "70%";
    }
    if(message.battery < 3.92 * 3){
      batPercent = "60%";
    }
    if(message.battery < 3.85 * 3){
      batPercent = "50%";
    }
    if(message.battery < 3.78 * 3){
      batPercent = "40%";
    }
    if(message.battery < 3.71 * 3){
      batPercent = "30%";
    }
    if(message.battery < 3.64 * 3){
      batPercent = "20%";
    }
    if(message.battery < 3.57 * 3){
      batPercent = "10%";
    }
    if(message.battery < 3.57 * 3){
      batPercent = "0%";
    }
    document.getElementById('batteryPercent').innerHTML = batPercent;
} 

});

// Connect to ROS
autoReconnect(function () {});
