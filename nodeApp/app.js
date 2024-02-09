var http = require('http');
var fs = require('fs');

var index = fs.readFileSync('index.html');

const { SerialPort } = require('serialport')
const { ReadlineParser } = require('@serialport/parser-readline')

var usbpath = '/dev/cu.usbmodem14101';


var port = new SerialPort({
    path: usbpath,
    baudRate: 9600,
    dataBits: 8,
    parity: 'none',
    stopBits: 1,
    flowControl: false
});

const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }))

var app = http.createServer(function(req, res) {
    res.writeHead(200, {'Content-Type': 'text/html'});
    res.end(index);
});

var io = require('socket.io')(app);

io.on('connection', function(socket) {
    console.log("Nodejs is listening");

    socket.on('acceleration', function(data) {
        port.write(data.status);
        console.log(data);
    });
});



parser.on('data', function(data) {
    console.log(data);

    io.emit('data', data);
    
});

app.listen(3000);