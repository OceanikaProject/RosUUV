ros_ip_address = '192.168.88.155'  


var ros = new ROSLIB.Ros({
  url : `ws://${ros_ip_address}:9090`
});

// map = function (x, in_min, in_max, out_min, out_max) {
//   return Math.floor((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
// }

// createLeftJoystick = function (data) {
//   var options = {
//     zone: document.getElementById('left_joystick'),
//     threshold: 0.1,
//     position: { left: '%' },
//     mode: 'static',
//     size: 100,
//     color: '#000000',
//   };

//   manager = nipplejs.create(options);

//   var lx = 511;
//   var ly = 511;

//   self.manager.on('start', function (event, nipple) {

//   });

//   self.manager.on('move', function (event, nipple) {
//     lx = Math.cos(nipple.angle.radian) * nipple.distance;
//     ly = Math.sin(nipple.angle.radian) * nipple.distance;
//     lx = map(lx, -50, 50, 0, 1023);
//     ly = map(ly, -50, 50, 0, 1023);
//   });

//   self.manager.on('end', function () {   
//     lx = 511;
//     ly = 511;
//   });

//   timer = setInterval(() => {
//       data.lx = lx;
//       data.ly = ly;
//     // console.log(`lx=${lx}, ly=${ly}`);
//   }, 10);
  
// }
// createRightJoystick = function (data) {
//   var options = {
//     zone: document.getElementById('right_joystick'),
//     threshold: 0.1,
//     position: { left: '%' },
//     mode: 'static',
//     size: 100,
//     color: '#000000',
//   };

//   manager = nipplejs.create(options);

//   rx = 511;
//   ry = 511;

//   self.manager.on('start', function (event, nipple) {

//   });

//   self.manager.on('move', function (event, nipple) {
//     rx = Math.cos(nipple.angle.radian) * nipple.distance;
//     ry = Math.sin(nipple.angle.radian) * nipple.distance;
//     rx = map(rx, -50, 50, 0, 1023);
//     ry = map(ry, -50, 50, 0, 1023);
//   });

//   self.manager.on('end', function () {
//     rx = 511;
//     ry = 511;
//   });

//   timer = setInterval(() => {
//     // console.log(`rx=${rx}, ry=${ry}`);
//     data.rx = rx;
//     data.ry = ry;
//   }, 10);
  
// }
 
// window.onload = function() {

  

//   var data = {
//     // lx: 511,
//     // ly: 511,
//     // rx: 511,
//     // ry: 511
//   };

//   var publisher = new ROSLIB.Topic({
//     ros : ros,
//     name : "/web_joystick",
//     messageType : 'std_msgs/String'
//   })
  
//   var video = document.getElementById('video');
//   video.src = "http://192.168.1.35:8081/stream?topic=raspicam_node/image"

//   createLeftJoystick(data);
//   createRightJoystick(data);

//   t = setInterval(() => {
//     console.log(data);
//     var msg = new ROSLIB.Message({
//         data : "Hello from roslibjs"
//     })
//     publisher.publish(msg);
//   }, 500);
// }


