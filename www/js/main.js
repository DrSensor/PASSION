// var chartGas = new google.visualization.LineChart($('#gas').get(0));	
// var dataGas = new google.visualization.DataTable();
// dataGas.addColumn('number', "CO"+"2".sub());
// dataGas.addColumn('number', "H"+"2".sub()+"S");

// var throttleGauge = new google.visualization.Gauge($('#gauge').get(0));
// var dataGauge = new google.visualization.DataTable();
// dataGauge.addColumn('number', "Throttle");

var viewCompass;
var viewTemperature;
var viewBattery;
var viewSignal;
var viewTachometer;
var viewMap;

var listenerCompass;
var listenerBattery;
var listenerSignal;
var listenerTemperature;
var listenerTachometer;
var listenerGas;
var navigation;

function initView() {
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

	viewMap = new ROS2D.Viewer({
		divID : 'map',
		width : 750,
		height : 800
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

	listenerTachometer = new ROSLIB.Topic({
		ros : ros,
		name : namespace + '/imu',
		messageType : 'sensor_msgs/Imu'
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

	listenerTachometer.subscribe(function(message) {
		viewTachometer.setOdoValue(odoValue++);
		viewTachometer.setValueAnimated(message.liniear.x*3.6);
		// listener.unsubscribe();
	});

	listenerGas.subscribe(function(message) {
		// dataGas.addRow([message.data]);
		// chartGas.draw(dataGas, {
		// 	title: "Gas"
		// });
		// // listener.unsubscribe();
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


