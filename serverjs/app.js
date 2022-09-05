const express = require('express');
const path = require('path');


const PORT = 8080;
const app = express();

app.use(express.static('static'));


app.get('/', (req, res) => {
    res.sendFile(path.resolve(__dirname, 'templates', 'main.html'));
});

app.get('/drone_description/meshes/base_link.STL', (req, res) => {
    console.log("base_link.STL");
    res.sendFile(path.resolve("/home/ubuntu/drone_ros/src/drone_description/meshes/base_link.STL"));
});

app.listen(PORT, () => {
    console.log(`Server started at ${PORT}`);
});