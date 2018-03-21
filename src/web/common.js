// Query parameters
urlParams = {};
(window.onpopstate = function () {
    var match,
        pl     = /\+/g,  // Regex for replacing addition symbol with a space
        search = /([^&=]+)=?([^&]*)/g,
        decode = function (s) { return decodeURIComponent(s.replace(pl, " ")); },
        query  = window.location.search.substring(1);

    urlParams = {};
    while (match = search.exec(query))
       urlParams[decode(match[1])] = decode(match[2]);
})();

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

state = State.clone();

// Connecting to ROS
// -----------------
ros = new ROSLIB.Ros();
ros.connected = false;

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
    this.connected = false;
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('error').style.display = 'inline';
    console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
    this.connected = true;
    console.log('Connection made!');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('error').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('connected').style.display = 'inline';
});

ros.on('close', function() {
    this.connected = false;
    console.log('Connection closed.');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'inline';
});

// Create a connection to the rosbridge WebSocket server.
// See if we're connected every once in a while and try to connect if we're not
function reconnect() {
    if (ros.connected != true) {
        console.log("Reconnecting")

		//if ("casas" in urlParams) {
		ros.connect('ws://'+window.location.hostname+':9090');
        //} else {
        //    ros.connect('ws://wsu-ras-joule.kyoto.local:9090');
        //}
    }
}
reconnect();
setInterval(reconnect, 10000);
