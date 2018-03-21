// Calling a service
// -----------------

tabletResponse = new ROSLIB.ActionClient({
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
        actionClient: tabletResponse,
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
    if (state.faceURL.length > 0) {
        document.getElementById("default-face").src = state.faceURL;
        document.getElementById("default-face").load();
    }
}
function showChoice() {
    showOne('choice');
    playSound('help-you.mp3');
    document.getElementById("face").src = state.faceURL;
}
function showOptions() {
    showOne('options');

    // Only show the "go to object" button if there is an object for this error
    if (state.objectName.length > 0) {
        document.getElementById("buttonGoTo").style.display = 'inline';
        document.getElementById("object").innerHTML = state.objectName
    } else {
        document.getElementById("buttonGoTo").style.display = 'none';
    }
}

// User response
function respondChoice(choice) {
    playSound('select-options.mp3');

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
            playVideo(state.videoFullURL);
            break;
        case "watchstep":
            playVideo(state.videoStepURL);
            break;
        case "goto":
            showDefault();
            break;
        case "complete":
            showDefault();
            break;
        default:
            console.log("Unknown option selection");
            break;
    }
}
function respondVideoDone() {
    sendROSResponse("videodone");
    showDefault();
}
function respondAudioDone() {
    sendROSResponse("audiodone");
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
document.getElementById("buttonComplete").onclick = function() {
    respondOptions("complete");
}
