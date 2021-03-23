
//////////////////////////////////////////////////////////////////////
//////////  Value Dictionaries ////////////////////////////////////////

SUBSCRIBERS = {
    "pulse": {
        data:[], topic_name:"/heart",
        sampleRate:100.0, sampleInterval:200,
    },
    "o2": {
        data:[], topic_name:"/spo2",
        sampleRate:100.0, sampleInterval:200,
    },
    "temp": {
        data:[], topic_name:"/temp",
        sampleRate:40.0, sampleInterval:200,
    }
}
SUBSCRIBER_TOPIC_PREFIX = "/biosensors"
SUBSCRIBERS_TOPIC_TYPE = "std_msgs/Float32MultiArray"

var tempBoundaries = [95, 99, 101]
var tempLabels = ['Low Temp', 'Average Temp', 'Possible Fever', 'Fever']
var tempValues = "90:120:2.5"

var pulseBoundaries = [60, 100, 130, 150]
var pulseLabels = ['Bradycardia', 'Resting HR', 'Heavy HR', 'Exercising HR', 'HR Too Rapid']
var pulseValues = "0:180:5.0"

var o2Boundaries = [90, 92, 94, 96]
var o2Labels = ['Not Enough O2', 'Low O2', 'Medium-Low O2', 'Medium HR', 'Healthy O2']
var o2Values = "0:100:2"


//////////////////////////////////////////////////////////////////////
//////////  feedback methods //////////////////////////////////////

function realTimeFeed(callback) {
	var tick = {};
	tick.plot0 = parseInt(10 + 90 * Math.random(), 10);
	tick.plot1 = parseInt(10 + 90 * Math.random(), 10);
	callback(JSON.stringify(tick));
};

function feedTemp(callback) {
	var tick = {};
	tick.plot0 = Math.trunc(SUBSCRIBERS["temp"].data[0]);
	callback(JSON.stringify(tick));
};

function feedO2(callback) {
	var tick = {};
	tick.plot0 = Math.trunc(SUBSCRIBERS["o2"].data[0]);
	callback(JSON.stringify(tick));
};

function feedPulse(callback) {
	var tick = {};
	tick.plot0 = Math.trunc(SUBSCRIBERS["pulse"].data[0]);
	callback(JSON.stringify(tick));
};

/////////////////////////////////////////////////////////////////
//////////  Global Configs //////////////////////////////////////

var _globals = {
	fontSize: 15
}
var _plotarea = {
	marginTop: 1
}
var _tooltip = {
	borderRadius: 5
}
var _indicator = [10, 10, 10, 10, 0.75];
var _animation = {
	effect: 2,
	method: 5,
	sequence: 4,
	speed: 900
}
var _scaleRItems = {
	offsetR: -15,
	angle: "auto",    //To adjust the angle of your scale labels.
	'font-size':12,
	rules: [{
		offsetX: 15
	}]
}
var _pretext = '%v<br><em>'


var _blue = '#0066cc';
var _green = '#33cc33';
var _orange = '#ff9933';
var _red = '#ff3300';
var _purple = '#6a0dad';

var _blueVal = '#0066cc';
var _redVal = '#ff3300';


/////////////////////////////////////////////////////////////////
//////////  Temp Config /////////////////////////////////////////

var tempConfig = {
	type: "gauge",
	backgroundColor: "#f8f9fa",
	globals: _globals,
	plotarea: _plotarea,
	plot: {
		size: '100%',
		valueBox: {
			placement: 'center',
			text: '%v', //default
			fontSize: 20,
			'border-width': 2,
			'border-color': "#0a101a",
			padding: "15%",
			color: "#f8f9fa",
			'border-radius': '4px',
			rules: [{
				rule: '%v >= ' + tempBoundaries[2],
				text: _pretext + tempLabels[3] + "</em>",
				'background-color': _red,
			},
			{
				rule: '%v < '  + tempBoundaries[2] + ' && %v >='  + tempBoundaries[1],
				text: _pretext + tempLabels[2] + "</em>",
				'background-color': _orange,
			},
			{
				rule: '%v < '  + tempBoundaries[1] + ' && %v >='  + tempBoundaries[0],
				text: _pretext + tempLabels[1] + "</em>",
				'background-color': _green,
			},
			{
				rule: '%v < ' + tempBoundaries[0],
				text: _pretext + tempLabels[0] + "</em>",
				'background-color': _blue,
			},
			]
		}
	},
	tooltip: _tooltip,
	scaleR: {
		aperture: 270,
		values: tempValues,
		center: { visible: false },
		tick: { visible: true },
		item: _scaleRItems,
		ring: {
			size: 50,
			rules: [{
				rule: '%v < ' + tempBoundaries[0],
				backgroundColor: _blue
			},
			{
				rule: '%v < '  + tempBoundaries[1] + ' && %v >='  + tempBoundaries[0],
				backgroundColor: _green
			},
			{
				rule: '%v < '  + tempBoundaries[2] + ' && %v >='  + tempBoundaries[1],
				backgroundColor: _orange
			},
			{
				rule: '%v >= ' + tempBoundaries[2],
				backgroundColor: _red
			}
			]
		}
	},
	refresh: {
		type: "feed",
		transport: "js",
		url: 'feedTemp()',
		interval: SUBSCRIBERS['temp'].sampleInterval,
	},
	series: [{
		values: [96], // starting value
		backgroundColor: 'black',
		indicator: _indicator,
		animation: _animation
	}]
};

///////////////////////////////////////////////////////////////
//////////  Pulse Config //////////////////////////////////////

var pulseConfig = {
	type: "gauge",
	backgroundColor: "#f8f9fa",
	globals: _globals,
	plotarea: _plotarea,
	plot: {
		size: '100%',
		valueBox: {
			placement: 'center',
			text: '%v', //default
			fontSize: 20,
			'border-width': 2,
			'border-color': "#0a101a",
			padding: "15%",
			color: "#f8f9fa",
			'border-radius': '4px',
			rules: [{
				rule: '%v >= ' + pulseBoundaries[3],
				text: _pretext + pulseLabels[4] + '</em>',
				'background-color': _purple,
			},
			{
				rule: '%v < '  + pulseBoundaries[3] + ' && %v >='  + pulseBoundaries[2],
				text: _pretext + pulseLabels[3] + "</em>",
				'background-color': _red,
			},
			{
				rule: '%v < '  + pulseBoundaries[2] + ' && %v >='  + pulseBoundaries[1],
				text: _pretext + pulseLabels[2]  + "</em>",
				'background-color': _orange,
			},
			{
				rule: '%v < '  + pulseBoundaries[1] + ' && %v >='  + pulseBoundaries[0],
				text: _pretext  + pulseLabels[1]  + "</em>",
				'background-color': _green,
			},
			{
				rule: '%v < ' + pulseBoundaries[0],
				text: _pretext + pulseLabels[0]  + "</em>",
				'background-color': _blue,
			},
			]
		}
	},
	tooltip: _tooltip,
	scaleR: {
		aperture: 270,
		values: pulseValues,
		center: { visible: false },
		tick: { visible: true },
		item: _scaleRItems,
		ring: {
			size: 50,
			rules: [
				{
				rule: '%v < ' + pulseBoundaries[0],
				backgroundColor: _blue
			},
			{
				rule: '%v < ' + pulseBoundaries[1] + ' && %v >= ' + pulseBoundaries[0],
				backgroundColor: _green
			},
			{
				rule: '%v < '  + pulseBoundaries[2] + ' && %v >='  + pulseBoundaries[1],
				backgroundColor: _orange
			},
			{
				rule: '%v < '  + pulseBoundaries[3] + ' && %v >='  + pulseBoundaries[2],
				backgroundColor: _red
			},
			{
				rule: '%v >= ' + pulseBoundaries[3],
				backgroundColor: _purple
			}
			]
		}
	},
	refresh: {
		type: "feed",
		transport: "js",
		url: 'feedPulse()',
		interval: SUBSCRIBERS['pulse'].sampleInterval,
	},
	series: [{
		values: [96], // starting value
		backgroundColor: 'black',
		indicator: _indicator,
		animation: _animation
	}]
};
	
///////////////////////////////////////////////////////////////
//////////  O2 Config /////////////////////////////////////////

var o2Config = {
	type: "gauge",
	backgroundColor: "#f8f9fa",
	globals: _globals,
	plotarea: _plotarea,
	plot: {
		size: '100%',
		valueBox: {
			placement: 'center',
			text: '%v', //default
			fontSize: 20,
			'border-width': 2,
			'border-color': "#0a101a",
			padding: "15%",
			color: "#f8f9fa",
			'border-radius': '4px',
			rules: [{
				rule: '%v >= ' + o2Boundaries[3],
				text: _pretext + o2Labels[4] + '</em>',
				'background-color': _purple,
			},
			{
				rule: '%v < '  + o2Boundaries[3] + ' && %v >='  + o2Boundaries[2],
				text: _pretext + o2Labels[3] + "</em>",
				'background-color': _red,
			},
			{
				rule: '%v < '  + o2Boundaries[2] + ' && %v >='  + o2Boundaries[1],
				text: _pretext + o2Labels[2]  + "</em>",
				'background-color': _red,
			},
			{
				rule: '%v < '  + o2Boundaries[1] + ' && %v >='  + o2Boundaries[0],
				text: _pretext  + o2Labels[1]  + "</em>",
				'background-color': _blue,
			},
			{
				rule: '%v < ' + o2Boundaries[0],
				text: _pretext + o2Labels[0]  + "</em>",
				'background-color': _blue,
			},
			]
		}
	},
	tooltip: _tooltip,
	scaleR: {
		aperture: 270,
		values: o2Values,
		center: { visible: false },
		tick: { visible: true },
		item: _scaleRItems,
		ring: {
			size: 50,
			"gradient-colors": _blueVal + " " + _redVal,  
			"gradient-stops":"0.0 1.0",  
			"fill-angle":0 
		}
	},
	refresh: {
		type: "feed",
		transport: "js",
		url: 'feedO2()',
		interval: SUBSCRIBERS['o2'].sampleInterval,
	},
	series: [{
		values: [50], // starting value
		backgroundColor: 'black',
		indicator: _indicator,
		animation: _animation
	}]
};

///////////////////////////////////////////////////////////////
//////////  Rendering ////////////////////////////////////////

function initSubscribers(ros) {
    for (var sensorKey in SUBSCRIBERS) {
        var sensorConfig = SUBSCRIBERS[sensorKey];

        let sub = new ROSLIB.Topic({
            ros : ros,
            name : SUBSCRIBER_TOPIC_PREFIX + sensorConfig.topic_name,
            messageType : SUBSCRIBERS_TOPIC_TYPE
        });

        let sensor = sensorKey

        sub.subscribe(function(message) {
			SUBSCRIBERS[sensor].data = message.data;
        });
        
        console.log("Initalized ", sensorKey, "(", sensor, ") Subscriber to ",  sub.name);
    }

	console.log("Subscribers initalized");

}

zingchart.render({
	id: 'temp_gauge',
	data: tempConfig,
	// height: 500,
	width: '100%'
});

zingchart.render({
	id: 'pulse_gauge',
	data: pulseConfig,
	// height: 500,
	width: '100%'
});

zingchart.render({
	id: 'o2_gauge',
	data: o2Config,
	// height: 500,
	width: '100%'
});