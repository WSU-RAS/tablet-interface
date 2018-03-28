// Calling a service
// -----------------
tabletResponse = new ROSLIB.ActionClient({
    ros : ros,
    serverName: '/tablet_response',
    actionName: 'ras_msgs/TabletAction'
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

// Commands for Web UI
// -------------------

function showOne(screen) {
    var screens = ['init','default','choice','options','video'];
    
    // Stop audio if currently playing
    var sound = document.getElementById("audio-wrapper");
    var asource = document.getElementById("audio-source");
    if (asource.getAttribute('src') != "") {
        sound.pause();
        asource.setAttribute('src', "");
    }

    // Stop video if currently playing
    var vid = document.getElementById("video-wrapper");
    var vsource = document.getElementById("video-source");
    if (vsource.getAttribute('src') != "") {
        vid.pause();
        vsource.setAttribute('src', "");
    }

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

function getVideoBasename() {
    // Kyoto connection too slow, so we host them on a local server there
    var basename;
    if (window.location.hostname == "wsu-ras-joule.kyoto.local") {
        basename = 'http://kyoto.kyoto.local/videos/'
    } else {
        basename = 'http://casas.wsu.edu/smarthomestats/video/'
    }
    return basename;
}

function getImageBasename() {
    // Kyoto connection too slow, so we host them on a local server there
    var basename;
    if (window.location.hostname == "wsu-ras-joule.kyoto.local") {
        basename = 'http://kyoto.kyoto.local/pictures/'
    } else {
        basename = 'http://casas.wsu.edu/smarthomestats/pictures/'
    }
    return basename;
}

function playVideo(url) {
    showOne('video');
    basename = getVideoBasename();

    var vid = document.getElementById("video-wrapper");
    var source = document.getElementById("video-source");
    source.setAttribute('src', basename + url);
    vid.onended = function() {
        respondVideoDone();
    };
    vid.load();
    vid.play();
    sendROSResponse("videoplay");
}

// Show screen
function showDefault(happy) {
    showOne('default');
    basename = getImageBasename();
    //if (state.faceURL.length > 0) {
    if (happy == true)
        document.getElementById("default-face").src = basename + 'blue_happy_with_mouth.jpg';
    else
        document.getElementById("default-face").src = basename + 'blue_happy_without_mouth.jpg';
    //document.getElementById("default-face").load();
    //}
}
function showChoice() {
    showOne('choice');
    playSound('resources/help-you.mp3');
    basename = getImageBasename();
    document.getElementById("face").src = basename + 'blue_surprised_with_mouth.jpg';
}
function showOptions() {
    showOne('options');
    playSound('resources/select-options.mp3');

    // Only show the "go to object" button if there is an object for this error
    if (state.objectName.length > 0) {
        document.getElementById("buttonGoTo").style.display = 'inline';
        document.getElementById("object").innerHTML = state.objectName;
    } else {
        document.getElementById("buttonGoTo").style.display = 'none';
    }
}

// User response
function respondChoice(choice) {
    var request;
    if (choice == true) {
        sendROSResponse("yes");
        showOptions();
    } else {
        sendROSResponse("no");
        showDefault(false);
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
            showDefault(false);
            // TODO play "Follow me"
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
    showDefault(false);
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
    showDefault(true); // Show happy face
}
document.getElementById("buttonInit").onclick = function() {
    showDefault(false);
    
    // This appears to fix the not playing on first call problem
    playSound("");
}

// Connect to ROS and automatically reconnect, creating the tablet service each time
autoReconnect(function () {
    // Advertising a Service
    // ---------------------
    // The Service object does double duty for both calling and advertising services
    var tablet = new ROSLIB.Service({
        ros : ros,
        name : '/tablet',
        serviceType : 'tablet_interface/Tablet'
    });

    // Use the advertise() method to indicate that we want to provide this service
    tablet.advertise(function(request, response) {
        var screen = request["screen"];

        // Update global state
        state = State.clone(request);

        // Show desired screen
        switch (screen) {
            case "default": showDefault(false); break;
            case "choice":  showChoice();  break;
            case "options": showOptions(); break;
            default:
                console.log("Unknown screen");
                break;
        }

        response['success'] = true;
        return true;
    });
});
