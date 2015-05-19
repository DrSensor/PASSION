// var throttleGauge = new google.visualization.Gauge($('#gauge').get(0));
// var dataGauge = new google.visualization.DataTable();
// dataGauge.addColumn('number', "Throttle");

var viewCompass;
var viewTemperature;
var viewBattery;
var viewSignal;
var viewTachometer;
var viewTrottle;
var viewGas;
var viewMap;

var listenerCompass;
var listenerBattery;
var listenerSignal;
var listenerTemperature;
var listenerIMU;
var listenerJoystick;
var listenerGas;
var navigation;

function initView() {
	///////////////////
	/// Steelseries //
	///////////////////
	// Define some sections
	var sections = [steelseries.Section(0, 25, 'rgba(0, 0, 220, 0.3)'),
		steelseries.Section(25, 50, 'rgba(0, 220, 0, 0.3)'),
		steelseries.Section(50, 75, 'rgba(220, 220, 0, 0.3)') ];
	// Define one area
	var areas = [steelseries.Section(75, 100, 'rgba(220, 0, 0, 0.3)')];
	// Define value gradient for bargraph
	var valGrad = new steelseries.gradientWrapper( 0,
		100,
		[ 0, 0.33, 0.66, 0.85, 1],
		[ new steelseries.rgbaColor(0, 0, 200, 1),
		new steelseries.rgbaColor(0, 200, 0, 1),
		new steelseries.rgbaColor(200, 200, 0, 1),
		new steelseries.rgbaColor(200, 0, 0, 1),
		new steelseries.rgbaColor(200, 0, 0, 1) ]
	);
	var odoValue = 99998.2;

	viewCompass = new steelseries.Compass('compass', {
		size: 140,
		rotateFace: true
	});

	viewTemperature = new steelseries.Linear('thermometer', {
		width: 140,
		height: 320,
		gaugeType: steelseries.GaugeType.TYPE2,
		titleString: "Temperature",
		unitString: "C",
		threshold: 70,
		lcdVisible: true
	});

	viewBattery = new steelseries.Battery('battery', {
		size: 80,
		value: 0
	});

	viewSignal = new steelseries.LinearBargraph('signal', {
		width: 320,
		height: 140,
		valueGradient: valGrad,
		useValueGradient: true,
		titleString: "Signal Strength",
		unitString: "%",
		threshold: 20,
		lcdVisible: true
	});

	viewTachometer = new steelseries.Radial('tachometer', {
		gaugeType: steelseries.GaugeType.TYPE4,
		size: 201,
		section: sections,
		area: areas,
		titleString: "Speed",
		unitString: "Km/h",
		threshold: 70,
		lcdVisible: true,
		lcdDecimals: 1,
		useOdometer: true,
		odometerParams: {digits: 5, value: odoValue}
	});

	///////////////
	/// Chartjs //
	///////////////
	var ctx = document.getElementById("gas").getContext("2d");
	var data = {
		labels: ["CO2", "H2S", "NH4"],
		datasets: [
			{
				label: "Max",
				fillColor: "rgba(220,220,220,0.2)",
				strokeColor: "rgba(220,220,220,1)",
				pointColor: "rgba(220,220,220,1)",
				pointStrokeColor: "#fff",
				pointHighlightFill: "#fff",
				pointHighlightStroke: "rgba(220,220,220,1)",
				data: []
			},
			{
				label: "Min",
				fillColor: "rgba(151,187,205,0.2)",
				strokeColor: "rgba(151,187,205,1)",
				pointColor: "rgba(151,187,205,1)",
				pointStrokeColor: "#fff",
				pointHighlightFill: "#fff",
				pointHighlightStroke: "rgba(151,187,205,1)",
				data: []
			}
		]
	};
	var options = {
		//Boolean - Whether to show lines for each scale point
		scaleShowLine : true,

		//Boolean - Whether we show the angle lines out of the radar
		angleShowLineOut : true,

		//Boolean - Whether to show labels on the scale
		scaleShowLabels : false,

		// Boolean - Whether the scale should begin at zero
		scaleBeginAtZero : true,

		//String - Colour of the angle line
		angleLineColor : "rgba(0,0,0,.1)",

		//Number - Pixel width of the angle line
		angleLineWidth : 1,

		//String - Point label font declaration
		pointLabelFontFamily : "'Arial'",

		//String - Point label font weight
		pointLabelFontStyle : "normal",

		//Number - Point label font size in pixels
		pointLabelFontSize : 12,

		//String - Point label font colour
		pointLabelFontColor : "#666",

		//Boolean - Whether to show a dot for each point
		pointDot : true,

		//Number - Radius of each point dot in pixels
		pointDotRadius : 3,

		//Number - Pixel width of point dot stroke
		pointDotStrokeWidth : 1,

		//Number - amount extra to add to the radius to cater for hit detection outside the drawn point
		pointHitDetectionRadius : 20,

		//Boolean - Whether to show a stroke for datasets
		datasetStroke : true,

		//Number - Pixel width of dataset stroke
		datasetStrokeWidth : 2,

		//Boolean - Whether to fill the dataset with a colour
		datasetFill : true,

		//String - A legend template
		legendTemplate : "<ul class=\"<%=name.toLowerCase()%>-legend\"><% for (var i=0; i<datasets.length; i++){%><li><span style=\"background-color:<%=datasets[i].strokeColor%>\"></span><%if(datasets[i].label){%><%=datasets[i].label%><%}%></li><%}%></ul>"
	};
	viewGas = new Chart(ctx).Radar(data, options);

	var target = document.getElementById('throttle'); // your canvas element
	var opts = {
		lines: 12, // The number of lines to draw
		angle: 0.15, // The length of each line
		lineWidth: 0.44, // The line thickness
		pointer: {
			length: 0.9, // The radius of the inner circle
			strokeWidth: 0.049, // The rotation offset
			color: '#000000' // Fill color
		},
		limitMax: 'false',   // If true, the pointer will not go past the end of the gauge
		colorStart: '#6FADCF',   // Colors
		colorStop: '#8FC0DA',    // just experiment with them
		strokeColor: '#E0E0E0',   // to see which ones work best for you
		generateGradient: true,
	};
	var viewTrottle = new Gauge(target).setOptions(opts); // create sexy gauge!
	viewTrottle.maxValue = 100; // set max gauge value
	viewTrottle.percentColors = [[0.0, "#a9d70b" ], [0.50, "#f9c802"], [1.0, "#ff0000"]];
	viewTrottle.set(0); // set actual value

	viewMap = new ROS2D.Viewer({
		divID : 'map',
		width : 320,
		height : 240
	});
}

function initROS(namespace) {
	var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
	});

	ros.on('connection', function() {
		console.log('Connected to websocket server.');
	});

	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
	});

	ros.on('close', function() {
		console.log('Connection to websocket server closed.');
	});

	listenerCompass = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/compass',
		messageType : 'std_msgs/UInt16'
	});

	listenerBattery = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/battery',
		messageType : 'std_msgs/UInt8'
	});

	listenerSignal = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/signal',
		messageType : 'std_msgs/UInt8'
	});

	listenerTemperature = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/temperature',
		messageType : 'std_msgs/Int16'
	});

	listenerIMU = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/imu',
		messageType : 'sensor_msgs/Imu'
	});

	listenerJoystick = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/joy',
		messageType : 'joy_msgs/joy'
	});

	listenerGas = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/CO2',
		messageType : 'std_msgs/UInt16'
	});

	navigation = NAV2D.OccupancyGridClientNav({
		ros : ros,
		rootObject : viewMap.scene,
		viewer : viewMap,
		serverName : namespace + '/pr2_move_base'
	});
}

function subscribe() {
	listenerCompass.subscribe(function(message) {
	viewCompass.setValueAnimated(message.data);
	// listener.unsubscribe();
	});

	listenerBattery.subscribe(function(message) {
		viewBattery.setValue(message.data);
		// listener.unsubscribe();
	});

	listenerSignal.subscribe(function(message) {
		viewSignal.setValueAnimated(message.data);
		// listener.unsubscribe();
	});

	listenerTemperature.subscribe(function(message) {
		viewTemperature.setValueAnimated(message.data);
		// listener.unsubscribe();
	});

	listenerIMU.subscribe(function(message) {
		viewTachometer.setOdoValue(odoValue++);
		viewTachometer.setValueAnimated(message.liniear.x*3.6);
				viewTrottle.set(0); // set actual value

		// listener.unsubscribe();
	});

	listenerJoystick.subscribe(function(message) {
		viewTrottle.set(message.analog.y); // set actual value
		// listener.unsubscribe();
	});

	listenerGas.subscribe(function(message) {
		viewGas.addData([message.data, message.data], "CO2");
		// listener.unsubscribe();
	});
}

initROS("passion");
subscribe();

// Scale the canvas to fit to the map
navigation.on('change', function(){
	viewMap.scaleToDimensions(navigation.currentGrid.width, navigation.currentGrid.height);
});

////////////////////////////
// GOOGLE CHART VIEW LOAD //
////////////////////////////

// // load chart lib
// google.load('visualization', '1', {
// 	packages: ['corechart','gauge']
// });

// // call drawChart once google charts is loaded
// google.setOnLoadCallback(drawChart);

  // Publishing a Topic
  // ------------------

  // var cmdVel = new ROSLIB.Topic({
  //   ros : ros,
  //   name : namespace + '/cmd_vel',
  //   messageType : 'geometry_msgs/Twist'
  // });

  // var twist = new ROSLIB.Message({
  //   linear : {
  //     x : 0.1,
  //     y : 0.2,
  //     z : 0.3
  //   },
  //   angular : {
  //     x : -0.1,
  //     y : -0.2,
  //     z : -0.3
  //   }
  // });
  // cmdVel.publish(twist);

// Subscribing to a Topic
// ----------------------



// Navigation Map
// ----------------------


