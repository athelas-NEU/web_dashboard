

// Audio
var audioCtx;
var source;
var stream;

// Button
var mute;

// Audio Nodes
var analyser;
var gainNode;


// Canvas
var canvas;
var canvasCtx;
var intendedWidth;
var drawAudio;

function initAudio() {
	initAudioNodes();
	initAudioCanvas();
	initAudioRecording();
}

function initAudioNodes() {
	audioCtx = new (window.AudioContext || window.webkitAudioContext)();
	mute = document.getElementById('audio_mute_off');

	analyser = audioCtx.createAnalyser();
	analyser.minDecibels = -90;
	analyser.maxDecibels = 90;
	analyser.smoothingTimeConstant = 0.1;
	
	gainNode = audioCtx.createGain();
}

function initAudioCanvas() {
	canvas = document.getElementById('audio_canvas');
	canvasCtx = canvas.getContext("2d");
	intendedWidth = document.getElementById('audio_wrapper').clientWidth;
	canvas.setAttribute('width', intendedWidth);
	drawAudio;
}

function initAudioRecording() {
	// Audio Recording
	if (navigator.mediaDevices.getUserMedia) {
	console.log('getUserMedia supported.');
	var constraints = { audio: true}
	navigator.mediaDevices.getUserMedia(constraints)
		.then(
			function (stream) {
				source = audioCtx.createMediaStreamSource(stream);
				source.connect(gainNode);
				gainNode.connect(analyser);

				console.log(source);

				analyser.connect(audioCtx.destination);
				visualize();
			})
		.catch(function (err) { console.log('The following gUM error occured: ' + err); })
	} else {
	console.log('getUserMedia not supported on your browser!');
	}

	mute.onclick = voiceMute;
}

function resumeAudio() {

	var uninitMsg = document.getElementById("audio_uninitalized");
	var waveformView = document.getElementById("audio_canvas");
	var waveforvmViewCtrl = document.getElementById("audio_canvas_controls");

	audioCtx.resume().then(() => {
		console.log('Playback resumed successfully');
		uninitMsg.style.display = "none";
		waveformView.style.display = "block";
		waveforvmViewCtrl.style.display = "block";
	  });
}

function voiceMuteActivate() {
	console.log("muting")
	gainNode.gain.setTargetAtTime(0, audioCtx.currentTime, 0)
	mute.id = "audio_mute_on";
	mute.innerHTML = "Unmute";
	mute.activated = true;
	mute.style.backgroundColor = "#ff3300"

	resumeAudio();
}

function voiceMute() {
	if (mute.id === "audio_mute_off") {
		voiceMuteActivate();
	} else {
		gainNode.gain.setTargetAtTime(1, audioCtx.currentTime, 0)
		mute.id = "audio_mute_off";
		mute.innerHTML = "Mute";
		mute.activated = false;
		mute.style.backgroundColor = "#0066cc"

		resumeAudio();
	}
}

function visualize() {
	var WIDTH = canvas.width;
	var HEIGHT = canvas.height;


	var visualSetting = "waveform";
	console.log(visualSetting);

	if (visualSetting === "waveform") {
		analyser.fftSize = 2048;
		var bufferLength = analyser.fftSize;
		var dataArray = new Uint8Array(bufferLength);

		canvasCtx.clearRect(0, 0, WIDTH, HEIGHT);

		var draw = function () {

			drawAudio = requestAnimationFrame(draw);

			analyser.getByteTimeDomainData(dataArray);

			canvasCtx.fillStyle = 'rgb(248, 249, 250)';
			canvasCtx.fillRect(0, 0, WIDTH, HEIGHT);

			canvasCtx.lineWidth = 2;
			canvasCtx.strokeStyle = 'rgb(51, 204, 51)';

			canvasCtx.beginPath();

			var sliceWidth = WIDTH * 1.0 / bufferLength;
			var x = 0;

			for (var i = 0; i < bufferLength; i++) {

				var v = dataArray[i] / 128.0;
				var y = v * HEIGHT / 2;

				if (i === 0) {
					canvasCtx.moveTo(x, y);
				} else {
					canvasCtx.lineTo(x, y);
				}

				x += sliceWidth;
			}

			canvasCtx.lineTo(canvas.width, canvas.height / 2);
			canvasCtx.stroke();
		};

		draw();


	} else if (visualSetting == "off") {
		canvasCtx.clearRect(0, 0, WIDTH, HEIGHT);
		canvasCtx.fillStyle = "red";
		canvasCtx.fillRect(0, 0, WIDTH, HEIGHT);
	}

}
