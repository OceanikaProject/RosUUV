
var coef_front_back = new ROSLIB.Param({
    ros : ros,
    name : "coef_front_back"
});
var pitch_resist_coef = new ROSLIB.Param({
    ros : ros,
    name : "pitch_resist_coef"
});
var up_down_power_coef = new ROSLIB.Param({
    ros : ros,
    name : "up_down_power_coef"
});
var coef_forward = new ROSLIB.Param({
    ros : ros,
    name : "coef_forward"
});
var coef_up = new ROSLIB.Param({
    ros : ros,
    name : "coef_up"
});
var coef_turn = new ROSLIB.Param({
    ros : ros,
    name : "coef_turn"
});
var vertical_left_reversed = new ROSLIB.Param({
    ros : ros,
    name : "vertical_left/reversed"
});
var vertical_right_reversed = new ROSLIB.Param({
    ros : ros,
    name : "vertical_right/reversed"
});
var horizontal_left_reversed = new ROSLIB.Param({
    ros : ros,
    name : "horizontal_left/reversed"
})
var horizontal_right_reversed = new ROSLIB.Param({
    ros : ros,
    name : "horizontal_right/reversed"
})
var vertical_back_reversed = new ROSLIB.Param({
    ros : ros,
    name : "vertical_back/reversed"
})

coef_front_back.get(function(value) {
    document.querySelector("#coef_front_back").value = value;
    document.querySelector("#coef_front_back_number").value = value;
});
pitch_resist_coef.get(function(value) {
    document.querySelector("#pitch_resist_coef").value = value;
    document.querySelector("#pitch_resist_coef_number").value = value;
});
up_down_power_coef.get(function(value) {
    document.querySelector("#up_down_power_coef").value = value;
    document.querySelector("#up_down_power_coef_number").value = value;
});
coef_forward.get(function(value) {
    document.querySelector("#coef_forward").value = value;
    document.querySelector("#coef_forward_number").value = value;
});
coef_up.get(function(value) {
    document.querySelector("#coef_up").value = value;
    document.querySelector("#coef_up_number").value = value;
});
coef_turn.get(function(value) {
    document.querySelector("#coef_turn").value = value;
    document.querySelector("#coef_turn_number").value = value;
});
vertical_left_reversed.get(function(value) {
    console.log("vertical_left_reversed", value);
    document.querySelector("#vertical_left_reversed").checked = value;
});
vertical_right_reversed.get(function(value) {
    console.log("vertical_right_reversed", value);
    document.querySelector("#vertical_right_reversed_hidden").checked = value;
});
horizontal_left_reversed.get(function(value) {
    console.log("horizontal_left_reversed", value);
    document.querySelector("#horizontal_left_reversed_hidden").checked = value;
});
horizontal_right_reversed.get(function(value) {
    console.log("horizontal_right_reversed", value);
    document.querySelector("#horizontal_right_reversed_hidden").checked = value;
});
vertical_back_reversed.get(function(value) {
    console.log("vertical_back_reversed", value);
    document.querySelector("#vertical_back_reversed_hidden").checked = value;
});   

document.querySelector("#oceanic_form").addEventListener("submit", function(e){
    e.preventDefault();
    coef_front_back.set(document.querySelector("#coef_front_back").value);
    pitch_resist_coef.set(document.querySelector("#pitch_resist_coef").value);
    up_down_power_coef.set(document.querySelector("#up_down_power_coef").value);
    coef_forward.set(document.querySelector("#coef_forward").value);
    coef_up.set(document.querySelector("#coef_up").value);
    coef_turn.set(document.querySelector("#coef_turn").value);
    vertical_left_reversed.set(document.querySelector("#vertical_left_reversed_hidden").checked === true);
    vertical_right_reversed.set(document.querySelector("#vertical_right_reversed_hidden").checked === true);
    horizontal_left_reversed.set(document.querySelector("#horizontal_left_reversed_hidden").checked === true);
    horizontal_right_reversed.set(document.querySelector("#horizontal_right_reversed_hidden").checked === true);
    vertical_back_reversed.set(document.querySelector("#vertical_back_reversed_hidden").checked === true);
});


PidDepth_P = new ROSLIB.Param({
    ros : ros,
    name: "PidDepth/P"
});
PidDepth_I = new ROSLIB.Param({
    ros : ros,
    name: "PidDepth/I"
});
PidDepth_D = new ROSLIB.Param({
    ros : ros,
    name: "PidDepth/D"
});
PidPitch_P = new ROSLIB.Param({
    ros : ros,
    name: "PidPitch/P"
});
PidPitch_I = new ROSLIB.Param({
    ros : ros,
    name: "PidPitch/I"
});
PidPitch_D = new ROSLIB.Param({
    ros : ros,
    name: "PidPitch/D"
});

PidDepth_P.get(function(value) {
    document.querySelector("#depthP").value = value;
    document.querySelector("#depthP_number").value = value;
});
PidDepth_I.get(function(value) {
    document.querySelector("#depthI").value = value;
    document.querySelector("#depthI_number").value = value;
});
PidDepth_D.get(function(value) {
    document.querySelector("#depthD").value = value;
    document.querySelector("#depthD_number").value = value;
});
PidPitch_P.get(function(value) {
    document.querySelector("#pitchP").value = value;
    document.querySelector("#pitchP_number").value = value;
});
PidPitch_I.get(function(value) {
    document.querySelector("#pitchI").value = value;
    document.querySelector("#pitchI_number").value = value;
});
PidPitch_D.get(function(value) {
    document.querySelector("#pitchD").value = value;
    document.querySelector("#pitchD_number").value = value;
});

document.querySelector("#pid_form").addEventListener("submit", function(e){
    e.preventDefault();
    PidDepth_P.set(document.querySelector("#depthP").value);
    PidDepth_I.set(document.querySelector("#depthI").value);
    PidDepth_D.set(document.querySelector("#depthD").value);
    PidPitch_P.set(document.querySelector("#pitchP").value);
    PidPitch_I.set(document.querySelector("#pitchI").value);
    PidPitch_D.set(document.querySelector("#pitchD").value);
});