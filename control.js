
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

STATE_SETTINGS = {
    "Idle": { description: "Select a test to complete and command the arm.", color: "#33cc33" },
    "PositionArmXY": { description: "Please wait as the arm aligns to the patient.", color: "#ff9933" },
    "PositionArmZ": { description: "Please wait as the arm aligns to the patient. Please stay stationary", color: "#ff9933" },
    "BioData": { description: "Collecting and visualizing vitals.", color: "#ff3300" },
    "Stop": { description: "Arm is stopped.", color: "#ff3300" },
    "StartUp" : { description: "Please make sure the athelas robot is fully booted. Then select a test to complete and command the arm.", color: "#0066cc"}
}

var stateSubscriber;

var robot_IP;
var ros;

//////////////// Helpers /////////////////////////

function formatDate(date) {
    var hours = date.getHours();
    var minutes = date.getMinutes();
    var seconds = date.getSeconds();
    var ampm = hours >= 12 ? 'pm' : 'am';
    hours = hours % 12;
    hours = hours ? hours : 12; // the hour '0' should be '12'
    minutes = minutes < 10 ? '0' + minutes : minutes;
    var strTime = hours + ':' + minutes + ':' + seconds + ' ' + ampm;
    return (date.getMonth() + 1) + "/" + date.getDate() + "/" + date.getFullYear() + "  " + strTime;
}

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
    if (publisher == null) {
        initPublishers()
        publisher = PUBLISHERS[name].pub;
    }
    return publisher;
}

//////////////// Node Initalizing ////////////////////

function initStateSubscriber() {
    stateSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: "/current_state",
        messageType: "std_msgs/String"
    });

    stateSubscriber.subscribe(function (message) {
        updateState(message.data);
    });

    console.log("Initalized state. Subscriber to ", stateSubscriber.name);
}

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
var stateLabel = document.getElementById("stateLabel")

var history_table_data = [
    { time: formatDate(new Date()), command: "Started system" }
];

var history_table = new Tabulator("#command-history", {
    data: history_table_data,           //load row data from array
    layout: "fitColumns",      //fit columns to width of table
    responsiveLayout: "hide",  //hide columns that dont fit on the table
    addRowPos: "top",          //when adding a new row, add it to the top of the table
    columns: [                 //define the table columns
        { title: "Date & Time", field: "time" },
        { title: "Command", field: "command" },
    ],
});



function updateLastCmd(_time, _command, color) {

    if (history_table_data.length >= 10) {
        history_table_data.shift();
    }
    history_table_data.unshift({
        time: formatDate(_time),
        command: _command
    })

    history_table.replaceData(history_table_data)
}

function updateState(stateName) {
    stateLabel.innerHTML = STATE_SETTINGS[stateName].description
    stateLabel.style.color = STATE_SETTINGS[stateName].color
}

function onClick_sensorSelect(sensor) {
    tempGauge.style.display = "none"
    pulseGauge.style.display = "none"
    o2Gauge.style.display = "none"

    if (false && sensor == "audio" || sensor == "stethoscope") {
        stethWave.style.display = "block"
        // voiceMuteActivate();
        return
    } else {
        // voiceMuteActivate();
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
    updateLastCmd(new Date(), "STARTED TEST, '" + sensor + "'", "#33cc33")
    publisher.publish(commandMsg);
}

function onClick_Halt() {
    var publisher = getPublisher("stop");
    commandMsg = new ROSLIB.Message({
        data: true
    });
    updateLastCmd(new Date(), "STOPPED ARM", "#ff3300")
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

    // determine robot address automatically
    robot_IP = location.hostname;
    // set robot address statically
    // robot_IP = "10.5.10.117";

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    // initAudio();
    initPublishers();
    initStateSubscriber();
    initSubscribers(ros);

    onClick_sensorSelect(getSensorSelection());
    updateState("StartUp");
    // voiceMuteActivate();

    // get handle for video placeholder
    video = document.getElementById('video');
    // Populate video source 
    video.src = "http://" + robot_IP + ":8080/stream?topic=/image&type=mjpeg&quality=80";
    video.onload = function () {
        alert("Video is initalized!");
    };
}