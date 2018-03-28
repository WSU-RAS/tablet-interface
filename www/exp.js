//Send task to Gabe
function sendTask(task_num, stat, stat_text){
  var sendTaskClient = new ROSLIB.Service({
    ros : ros,
    name : '/task_controller',
    serviceType : 'ras_msgs/TaskController'
  });

  var request = new ROSLIB.ServiceRequest({
    //name : task
    id : {
        task_number : task_num
    },
    request : {
        status : stat,  // START=2, END=3
        text : stat_text
    }
  });

  // Finally, we call the /add_two_ints service and get back the results in the callback. The result
  // is a ROSLIB.ServiceResponse object.
  sendTaskClient.callService(request, function(result) {
    console.log('Result for service call on ' + sendTaskClient.name + ': ' + result.status.status + ', ' + result.status.text);
    //On success
    if (result.status.status == 4){
      document.getElementById('status').innerHTML = '<span class="badge badge-success">REQUEST SUCCESSFUL</span>';
      if (request.request.status == 3) {
        document.getElementById('task_name').innerHTML = '<span class="badge badge-secondary">' + request.request.text + '</span>';
        // Stop any task
        document.getElementById('takeMedStart').style.display = 'inline';
        document.getElementById('takeMedStop').style.display = 'none';
        document.getElementById('walkDogStart').style.display = 'inline';
        document.getElementById('walkDogStop').style.display = 'none';
        document.getElementById('waterPlantsStart').style.display = 'inline';
        document.getElementById('waterPlantsStop').style.display = 'none';
      }
      else {
        document.getElementById('task_name').innerHTML = '<span class="badge badge-primary">' + request.request.text + '</span>';
        if (request.id.task_number == 0) {
          // Water Plants
          document.getElementById('takeMedStart').style.display = 'none';
          document.getElementById('takeMedStop').style.display = 'none';
          document.getElementById('walkDogStart').style.display = 'none';
          document.getElementById('walkDogStop').style.display = 'none';
          document.getElementById('waterPlantsStart').style.display = 'none';
          document.getElementById('waterPlantsStop').style.display = 'inline';
        }
        else if (request.id.task_number == 1) {
          // Take Meds
          document.getElementById('takeMedStart').style.display = 'none';
          document.getElementById('takeMedStop').style.display = 'inline';
          document.getElementById('walkDogStart').style.display = 'none';
          document.getElementById('walkDogStop').style.display = 'none';
          document.getElementById('waterPlantsStart').style.display = 'none';
          document.getElementById('waterPlantsStop').style.display = 'none';
        }
        else if (request.id.task_number == 2) {
          // Walk Dog
          document.getElementById('takeMedStart').style.display = 'none';
          document.getElementById('takeMedStop').style.display = 'none';
          document.getElementById('walkDogStart').style.display = 'none';
          document.getElementById('walkDogStop').style.display = 'inline';
          document.getElementById('waterPlantsStart').style.display = 'none';
          document.getElementById('waterPlantsStop').style.display = 'none';
        }
      }
    }
    //On Failure
    if (result.status.status == 5){
        document.getElementById('status').style.color = '#FF0000';
        document.getElementById('status').innerHTML = '<span class="badge badge-danger">REQUEST FAILED</span>';
        document.getElementById('takeMedStart').style.display = 'inline';
        document.getElementById('takeMedStop').style.display = 'inline';
        document.getElementById('walkDogStart').style.display = 'inline';
        document.getElementById('walkDogStop').style.display = 'inline';
        document.getElementById('waterPlantsStart').style.display = 'inline';
        document.getElementById('waterPlantsStop').style.display = 'inline';
    }

  });
}

// Selecting buttons
document.getElementById("takeMedStart").onclick = function() {
    sendTask(1, 2, "START TAKE MEDS TASK");
}
document.getElementById("takeMedStop").onclick = function() {
    sendTask(1, 3, "END TAKE MEDS TASK");
}
document.getElementById("walkDogStart").onclick = function() {
    sendTask(2, 2, "START WALK DOG TASK");
}
document.getElementById("walkDogStop").onclick = function() {
    sendTask(2, 3, "STOP WALK DOG TASK");
}
document.getElementById("waterPlantsStart").onclick = function() {
    sendTask(0, 2, "START WATER PLANTS TASK");
}
document.getElementById("waterPlantsStop").onclick = function() {
    sendTask(0, 3, "STOP WATER PLANTS TASK");
}


//Access Battery Information
var globalTime = new Date();
var timeHolder = globalTime.getTime();

var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/sensor_state',
  messageType : 'turtlebot3_msgs/SensorState'
});

var battery_replace = false;

listener.subscribe(function(message) {
  var testTime = new Date();
  var testTime2 = testTime.getTime();
  var batPercent = "";



  if(timeHolder + 1000 < testTime2)
  {
    globalTime = new Date();
    timeHolder = globalTime.getTime();
    document.getElementById('battery').innerHTML = '<span class="badge badge-info">' + message.battery.toFixed(2) + "V" + '</span>';

    if(message.battery > 4.2 * 3){
      batPercent = "100%";
      battery_replace = false;
    }
    if(message.battery < 4.13 * 3){
      batPercent = "85%";
      battery_replace = false;
    }
    if(message.battery < 4.06 * 3){
      batPercent = "60%";
      battery_replace = false;
    }
    if(message.battery < 3.99 * 3){
      batPercent = "45%";
      battery_replace = false;
    }
    if(message.battery < 3.92 * 3){
      batPercent = "30%";
      battery_replace = false;
    }
    if(message.battery < 3.85 * 3){
      batPercent = "15%";
      battery_replace = false;
    }
    if(message.battery < 3.78 * 3){
      batPercent = "5%";
      if (battery_replace == false){
        battery_replace = true;
        alert("Please replace the battery! Plug Joule into wall and swap battery.");
      }
    }
    if(message.battery < 3.71 * 3){
      batPercent = "00%";
    }
    if(message.battery < 3.64 * 3){
      batPercent = "-10%";
    }
    if(message.battery < 3.57 * 3){
      batPercent = "-20%";
    }
    if(message.battery < 3.57 * 3){
      batPercent = "-30%";
    }
    document.getElementById('batteryPercent').innerHTML = '<span class="badge badge-info">' + batPercent + '</span>';
}

});

/**
* Setup all visualization elements when the page is loaded.
*/
function viz() {
    document.getElementById("map").innerHTML = "";
    document.getElementById("mjpeg").innerHTML = "";

    // Create the main viewer.
    var mapviewer = new ROS2D.Viewer({
      divID : 'map',
      width : 640, 
      height : 480
    });

    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : mapviewer.scene,
      viewer : mapviewer,
      serverName : '/move_base',
      image: 'resources/turtlebot.png'
    });

    /*
    // Setup the map client.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : mapviewer.scene,
      // Use this property in case of continuous updates			
      continuous: true
    });
    // Scale the canvas to fit to the map
    gridClient.on('change', function() {
      mapviewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
      mapviewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
    });
    */
    // Create the main viewer.
    var camviewer = new MJPEGCANVAS.Viewer({
      divID : 'mjpeg',
      host : window.location.hostname,
      width : 640,
      height : 480,
      topic : '/detection_image',
      interval : 1000
    });
}

// Connect to ROS - show camera/map stuff on connect
autoReconnect(function () { }, function() { viz() });
