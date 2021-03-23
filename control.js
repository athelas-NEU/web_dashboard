
PUBLISHERS = {
    "runTest": {
        id: "command_RunTest", pub: null,
        topic: { name: "/start_test", type: "std_msgs/String" }
    },

    "returnToHome": {
        id: "command_ReturnToHome", pub: null,
        topic: { name: "/arm_control/reset", type: "std_msgs/Bool" },
    },

    "stop": {
        id: "command_Halt", pub: null,
        topic: { name: "/arm_control/stop", type: "std_msgs/Bool" },
    },
}

var robot_IP;
var ros;

//////////////// Helpers /////////////////////////

function getSensorSelection() {
    var radios = document.getElementsByName('sensorSelect');
    var sensor = "stethoscope"
    for (var i = 0, length = radios.length; i < length; i++) {
        if (radios[i].checked) {
            sensor = radios[i].value;
            break;
        }
    }

    return sensor;
}

function getPublisher(name) {
    var publisher = PUBLISHERS[name].pub;
    if(publisher == null) {
        initPublishers()
        publisher = PUBLISHERS[name].pub;
    }
    return publisher;
}

//////////////// Node Initalizing ////////////////////

function initPublishers() {
    for (var cmdKey in PUBLISHERS) {
        var cmdConfig = PUBLISHERS[cmdKey];
        var publisher = new ROSLIB.Topic({
            ros: ros,
            name: cmdConfig.topic.name,
            messageType: cmdConfig.topic.type
        });

        cmdConfig.pub = publisher;
        cmdConfig.pub.advertise();

        console.log("Initalized ", cmdKey, " Publisher to ", cmdConfig.topic.name);
    }

    console.log("Publishers initalized");
}

//////////////////// Controlling ///////////////////////

var tempGauge = document.getElementById("temp_wrapper")
var pulseGauge = document.getElementById("pulse_wrapper")
var o2Gauge = document.getElementById("o2_wrapper")
var stethWave = document.getElementById("audio_wrapper")
var lastCmdLabel = document.getElementById("lastCmdLabel")

function updateLastCmd(time, command, color){
    lastCmdLabel.innerHTML = "[" + time.toLocaleString() + ", " + command + "]"
    lastCmdLabel.style.color = color
}

function onClick_sensorSelect(sensor) {
    tempGauge.style.display = "none"
    pulseGauge.style.display = "none"
    o2Gauge.style.display = "none"

    if(sensor == "audio" || sensor == "stethoscope"){
        stethWave.style.display = "block"
        voiceMuteActivate();
        return
    } else {
        voiceMuteActivate();
        stethWave.style.display = "none"
    }

    document.getElementById(sensor + "_wrapper").style.display = "block"
}

function onClick_RunTest() {
    var publisher = getPublisher("runTest");
    var sensor = getSensorSelection();
    onClick_sensorSelect(sensor)
    commandMsg = new ROSLIB.Message({
        data: sensor
    });
    updateLastCmd(new Date(), "START TEST, '" + sensor + "'" , "#33cc33")
    publisher.publish(commandMsg);
}

function onClick_Halt() {
    var publisher = getPublisher("stop");
    commandMsg = new ROSLIB.Message({
        data: true
    });
    updateLastCmd(new Date(), "STOP ARM", "#ff3300")
    publisher.publish(commandMsg);
}

function onClick_ReturnHome() {
    var publisher = getPublisher("returnToHome");
    commandMsg = new ROSLIB.Message({
        data: true
    });
    updateLastCmd(new Date(), "RESET ARM", "#0066cc")
    publisher.publish(commandMsg);
}


window.onload = function () {
    /*
    Choose between static IP address for the robot or dynamically defined IP 
    with location.hostname depending on your configuration. If the web server
    is running on a device different than the robot, IP must be set manually
    to the robot address. If the app is deployed to a server which is running
    on your robot, it can be set automatically.
    */
    onClick_sensorSelect(getSensorSelection());

    // determine robot address automatically
    robot_IP = location.hostname;
    // set robot address statically
    // robot_IP = "10.5.10.117";

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    initPublishers();
    initSubscribers(ros);

    voiceMuteActivate();

    // get handle for video placeholder
    video = document.getElementById('video');
    // Populate video source 
    // video.src = "http://" + robot_IP + ":8080/stream?topic=/camera/rgb/image_raw&type=mjpeg&quality=80";
    // video.onload = function () {
    //     alert("Video is initalized!");
    // };
}