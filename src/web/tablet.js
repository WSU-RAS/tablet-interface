// State
//
// I've never done Javascript objects before, so this may not be the best way
// to do it.
//
// https://gist.github.com/hallettj/64478
if (typeof Object.create !== 'function') {
    Object.create = function(o) {
        var F = function() {};
        F.prototype = o;
        return new F();
    };
}

var State = {
    clone: function(d) {
        var newState = Object.create(this);

        if (d !== undefined) {
            newState.screen = d["screen"];
            newState.objectName = d["object"];
            newState.faceURL = d["face_url"];
            newState.videoStepURL = d["video_step_url"];
            newState.videoFullURL = d["video_full_url"];
            newState.audioURL = d["audio_url"];
        } else {
            newState.screen = "default";
            newState.objectName = "";
            newState.faceURL = "";
            newState.videoStepURL = "";
            newState.videoFullURL = "";
            newState.audioURL = "";
        }
        return newState;
    }
};

this.state = State.clone();

// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('error').style.display = 'inline';
    console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
    console.log('Connection made!');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('error').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('connected').style.display = 'inline';
});

ros.on('close', function() {
    console.log('Connection closed.');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'inline';
});

// Create a connection to the rosbridge WebSocket server.
//
// For this demo, we'll assume it's on localhost.
//ros.connect('ws://brian-gpu.ailab.wsu.edu:9090');
ros.connect('ws://wsu-ras-joule.ailab.wsu.edu:9090');

// Calling a service
// -----------------

this.tabletResponse = new ROSLIB.ActionClient({
    ros : ros,
    serverName: '/tablet_response',
    actionName: 'ras_msgs/TabletGotoAction'
});

// Advertising a Service
// ---------------------

// The Service object does double duty for both calling and advertising services
var setBoolServer = new ROSLIB.Service({
    ros : ros,
    name : '/tablet',
    serviceType : 'tablet_interface/Tablet'
});

// Note: this will later be some sort of action but I'll change that later
function sendROSResponse(msg) {
    var goal = new ROSLIB.Goal({
        actionClient: this.tabletResponse,
        goalMessage: {
            response: msg
        }
    });

    goal.on('feedback', function(feedback) {
        console.log('Feedback: ' + feedback.text);
    });

    goal.on('result', function(result) {
        console.log('Result: ' + result.success);
    });

    goal.send();
}

// Use the advertise() method to indicate that we want to provide this service
setBoolServer.advertise(function(request, response) {
    var screen = request["screen"];

    // Update global state
    state = State.clone(request);

    // Show desired screen
    switch (screen) {
        case "default": showDefault(); break;
        case "choice":  showChoice();  break;
        case "options": showOptions(); break;
        default:
            console.log("Unknown screen");
            break;
    }

    response['success'] = true;
    return true;
});

// Commands for Web UI
// -------------------

function showOne(screen) {
    var screens = ['default','choice','options','video'];
    
    var sound = document.getElementById("audio-wrapper");
    sound.pause();
    var asource = document.getElementById("audio-source");
    asource.setAttribute('src', "");
    var vid = document.getElementById("video-wrapper");
    vid.pause();
    var vsource = document.getElementById("video-source");
    vsource.setAttribute('src', "");

    // Hide all others
    for (var i = 0; i < screens.length; ++i)
        if (screens[i] != screen)
            document.getElementById(screens[i]).style.display = 'none';

    // Make sure the one we want is showing
    if (screen !== undefined)
        document.getElementById(screen).style.display = 'inline';
}


// Multimedia
function playSound(audioURL) {
    
    var sound = document.getElementById("audio-wrapper");
    var source = document.getElementById("audio-source");
    source.setAttribute('src', audioURL);
    sound.onended = function() {
        respondAudioDone();
    };
    sound.load();
    sound.play();
    sendROSResponse("audioplay");
}

function playVideo(url) {
    showOne('video');

    var vid = document.getElementById("video-wrapper");
    var source = document.getElementById("video-source");
    source.setAttribute('src', url);
    vid.onended = function() {
        respondVideoDone();
    };
    vid.load();
    vid.play();
    sendROSResponse("videoplay");
}

// Show screen
function showDefault() {
    showOne('default');
    if (this.state.faceURL.length > 0) {
        document.getElementById("default-face").src = this.state.faceURL;
        document.getElementById("default-face").load();
    }
}
function showChoice() {
    showOne('choice');
    playSound(this.state.audioURL);
    document.getElementById("face").src = this.state.faceURL;
}
function showOptions() {
    showOne('options');

    // Only show the "go to object" button if there is an object for this error
    if (this.state.objectName.length > 0) {
        document.getElementById("buttonGoTo").style.display = 'inline';
        document.getElementById("object").innerHTML = this.state.objectName
    } else {
        document.getElementById("buttonGoTo").style.display = 'none';
    }
}

// User response
function respondChoice(choice) {
    playSound(this.state.audioURL);

    var request;
    if (choice == true) {
        sendROSResponse("yes");
        showOptions();
    } else {
        sendROSResponse("no");
        showDefault();
    }
}
function respondOptions(option) {
    sendROSResponse(option);

    switch (option) {
        case "watchfull":
            playVideo(this.state.videoFullURL);
            break;
        case "watchstep":
            playVideo(this.state.videoStepURL);
            break;
        case "goto":
            // Either here show the options screen and play the "follow me"
            // sound or let the manager send another command to play that sound
            // file and show the options screen
            //
            // For now, just show default screen:
            showDefault();
            break;
        default:
            console.log("Unknown option selection");
            break;
    }
}
function respondVideoDone() {
    sendROSResponse("videodone");

    // Probably go back to options? Maybe manager node will send another
    // command.
    showDefault();
}
function respondAudioDone() {
    sendROSResponse("audiodone");

    // Probably go back to options? Maybe manager node will send another
    // command.
}

// Selecting buttons
document.getElementById("buttonYes").onclick = function() {
    respondChoice(true);
}
document.getElementById("buttonNo").onclick = function() {
    respondChoice(false);
}
document.getElementById("buttonVideoFull").onclick = function() {
    respondOptions("watchfull");
}
document.getElementById("buttonVideoStep").onclick = function() {
    respondOptions("watchstep");
}
document.getElementById("buttonGoTo").onclick = function() {
    respondOptions("goto");
}
