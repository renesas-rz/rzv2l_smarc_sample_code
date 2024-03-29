/*
 * websocket_demo.js
 *
 * Copyright (c) 2019 Renesas Electronics Corp.
 * This software is released under the MIT License,
 * see https://opensource.org/licenses/MIT
 */

// let socket = new WebSocket('ws://localhost:3000/ws/', 'graph-update');
let socket = new WebSocket('ws://192.168.1.11:3000/ws/');
let predCanvas = document.getElementById('pred_canvas');
let defaultCanvas = document.getElementById('default_canvas');
let predCtx = document.getElementById('pred_canvas').getContext('2d');
let defaultCtx = document.getElementById('default_canvas').getContext('2d');
let graphCtx = document.getElementById('graph_canvas').getContext('2d');
let nowTimes = document.getElementsByClassName('now_time');
let predWindowData = document.getElementById('pred_window');
let drpWindowData = document.getElementById('drp_window');

defaultCtx.fillStyle = 'darkgray';
defaultCtx.fillRect(0, 0, 640, 480);
defaultCtx.font = '12pt Arial';

predCtx.fillStyle = 'darkgray';
predCtx.fillRect(0, 0, 640, 480);
predCtx.font = '12pt Arial';

let startTime = null;

let postProcessTime = null;
let cam_postProcessTime = null;
let renderingTime = 0.0;

let endTime = moment();
let camera_endTime = moment();
let totalTime = 0.0;

let totalSpeed = 0.0;

let predTime = 0.0;
let camTime = 0.0;

let webcam = new Image();
let orgcam = new Image();

let graphChart = new Chart(graphCtx, {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      {
        label: 'CPU 0 Usage[%]',
        data: [],
        borderColor: 'blue',
        backgroundColor: 'rgba(0,0,0,0)'
      },
      {
        label: 'CPU 1 Usage[%]',
        data: [],
        borderColor: 'deepskyblue',
        backgroundColor: 'rgba(0,0,0,0)'
      }
    ],
  },
  options: {
    scales: {
      y: {
        type: 'linear',
        min: 0,
        max: 100
      }
    },
    animation: {
      duration: 0
    }
  }
}
);

// Send Command: Model Change
let model = document.getElementById('change_model');
model.addEventListener('change', inputChange);

function inputChange(event) {
  // console.log(event.currentTarget.value);
  socket.send(JSON.stringify({
    command_name: 'change_model',
    Value: {
      model: event.currentTarget.value
    }
  }));
}

// Pose Estimation: Line Drawing
function drawLine(ctx, pts, start, end, ratio_x, ratio_y) {
  if ((pts[start] === null) || (pts[end] === null)) {
    return;
  }
  else if ((pts[start] === undefined) || (pts[end] === undefined)) {
    return;
  }
  else if ((pts[start] === 0) && pts[end] === 0) {
    return;
  }
  else {
    ctx.moveTo((pts[start].X) * ratio_x, (pts[start].Y) * ratio_y); // start point
    ctx.lineTo((pts[end].X) * ratio_x, (pts[end].Y) * ratio_y); // end point
    ctx.stroke(); // drawing
    return;
  }
}

// Pose Estimation: Point Drawing
function drawKeyPoint(ctx, pts, ratio_x, ratio_y) {
  ctx.beginPath();
  ctx.arc((pts.X) * ratio_x, (pts.Y) * ratio_y, 3, 0, (2 * Math.PI));
  ctx.closePath();
  ctx.fill();
}

// USB Camera Image Process Time Measurement
function measureCameraTime(ctx, nowTime) {
  cam_postProcessTime = moment();
  camTime = (1000 / cam_postProcessTime.diff(camera_endTime)).toFixed(3);
  ctx.fillText('fps: ' + camTime, 540, 20);
  camera_endTime = moment();
}

// Process Time Measurement
function measureProcessingTime(ctx, nowTime) {
  // console.log('----------------------------------------');
  postProcessTime = moment();
  // console.log('postProcessTime ' + postProcessTime);

  // Time from JSON receive -> Finish drawing
  renderingTime = postProcessTime.diff(nowTime);
  // Total Time
  totalTime = postProcessTime.diff(endTime);
  // Process Speed
  totalSpeed = (1000 / totalTime);

  ctx.fillText('Rendering time: ' + renderingTime + ' ms', 470, 20);
  ctx.fillText('Total time:     ' + totalTime + ' ms', 470, 40);
  ctx.fillText('fps:                    ' + totalSpeed.toFixed(3), 470, 60);

  // console.log('renderingTime ' + renderingTime);
  // console.log('totalTime ' + totalTime);
  // console.log('totalSpeed ' + totalSpeed);

  endTime = moment();
  // console.log('endTime ' + endTime);
  // console.log('----------------------------------------');
}

$(() => {
  socket.onmessage = function (event) {
    // Calculate process time
    let nowTime = moment();
    if (startTime === null) {
      startTime = moment();
    }
    let cnt = nowTime.diff(startTime, 'seconds');
    let elapsedTime = moment('2022-04-01 00:00:00').add(cnt, 's').format('HH:mm:ss');
    nowTimes[0].innerHTML = elapsedTime;
    nowTimes[1].innerHTML = elapsedTime;

    // JSON parse
    let datas = JSON.parse(event.data);
    console.log(datas);

    let predData;
    let predDatas = [];
    let predStr;
    let drpData = null;
    let len;
    let i = 0;

    // Update CPU Usage Graph
    if (datas.command_name === 'cpu_usage') {
      graphChart.data.labels.push(nowTime.format('HH:mm:ss'));
      let cpuUsages = datas.Value.cpu_usage.split(' ');
      graphChart.data.datasets[0].data.push(cpuUsages[0]);
      graphChart.data.datasets[1].data.push(cpuUsages[1]);

      if (graphChart.data.labels.length > 10) {
        graphChart.data.labels.shift();
        graphChart.data.datasets.forEach((dataset) => {
          dataset.data.shift();
        });
      }
      graphChart.update();
    }
    else if (datas.command_name === 'camera_image') {
      orgcam.src = 'data:image/jpeg;base64,' + datas.Value.img;

      orgcam.onload = function () {
        if (datas.Value.img) {
          defaultCtx.drawImage(orgcam, 0, 0, 640, 480);

          measureCameraTime(defaultCtx, nowTime);
        }
      }
    }
    // YOLOv3, TinyYOLOv2
    else if (datas.command_name === 'object_detection') {
      predCtx.linewidth = 8;
      predCtx.strokeStyle = 'blue';
      predCtx.fillStyle = 'blue';
      defaultCtx.fillStyle = 'blue';

      predData = datas.Value.predict;
      len = predData.length;

      let cls = [];
      let pred = [];
      let x = [];
      let y = [];
      let w = [];
      let h = [];

      for (i = 0; i < len; i++) {
        predStr = predData[i];
        cls[i] = String(predStr.name);
        pred[i] = Number.parseFloat(predStr.pred).toFixed(2);
        x[i] = Number(predStr.X);
        y[i] = Number(predStr.Y);
        w[i] = Number(predStr.W);
        h[i] = Number(predStr.H);

        if (i !== 0) {
          predDatas[i] = '\n' + cls[i] + ' :\t' + pred[i] + ' %\n' + 'X: ' + x[i] + '\tY: ' + y[i] + '\tW: ' + w[i] + '\tH: ' + h[i];
        }
        else {
          predDatas[i] = cls[i] + ' :\t' + pred[i] + ' %\n' + 'X: ' + x[i] + '\tY: ' + y[i] + '\tW: ' + w[i] + '\tH: ' + h[i];
        }
      }

      webcam.src = 'data:image/jpeg;base64,' + datas.Value.img;


      let ratio_w = predCanvas.width / datas.Value.img_org_w;
      let ratio_h = predCanvas.height / datas.Value.img_org_h;
      console.log('ratio_w ' + ratio_w);
      console.log('ratio_h ' + ratio_h);



      webcam.onload = function () {
        // Output USB camera image
        predCtx.drawImage(webcam, 0, 0, predCanvas.width, predCanvas.height);

        for (i = 0; i < len; i++) {
          predCtx.strokeRect(x[i] * ratio_w, y[i] * ratio_h, w[i] * ratio_w, h[i] * ratio_h);
          predCtx.fillText(cls[i], x[i] * ratio_w, (y[i] * ratio_h + h[i] * ratio_h + 16));
        }

        measureProcessingTime(predCtx, nowTime);
      }

      // Calculate & Display DRP process time 
      drpData = 'Inference time(DRP-AI):' + '\t\t' + datas.Value.drp_time + ' ms\n' +
        'Post-processing time(CPU):' + '\t' + datas.Value.post_time + ' ms\n';

      predWindowData.value = predDatas;
      drpWindowData.value = drpData;
    }
    // HRNet
    else if (datas.command_name === 'pose_detection') {
      predCtx.linewidth = 8;
      predCtx.strokeStyle = 'yellow';
      predCtx.fillStyle = 'yellow';
      defaultCtx.fillStyle = 'yellow';

      predData = datas.Value.predict;
      len = predData.length;

      for (i = 0; i < len; i++) {
        if (i !== 0) {
          predDatas[i] = '\n' + 'No.' + (i + 1) + '\t' + 'X: ' + predData[i].X + '\tY: ' + predData[i].Y;
        }
        else {
          predDatas[i] = 'No.' + (i + 1) + '\t' + 'X: ' + predData[i].X + '\tY: ' + predData[i].Y;
        }
      }


      webcam.src = 'data:image/jpeg;base64,' + datas.Value.img;
      let ratio_w = predCanvas.width / datas.Value.img_org_w;
      let ratio_h = predCanvas.height / datas.Value.img_org_h;

      webcam.onload = function () {
        // Display Image
        predCtx.drawImage(webcam, 0, 0, predCanvas.width, predCanvas.height);

        // Draw Inference Crop Range (VGA: X = 185/ Y = 0/ Width = 270/ Height = 480)
        predCtx.strokeRect(185 * ratio_w, 0 * ratio_h, 270 * ratio_w, 480 * ratio_h);
        predCtx.fillText("Please stand here", (185 + 5) * ratio_w, (datas.Value.img_org_h - 5) * ratio_h);

        // Draw Skeleton
        drawLine(predCtx, predData, 0, 1, ratio_w, ratio_h);
        drawLine(predCtx, predData, 1, 2, ratio_w, ratio_h);
        drawLine(predCtx, predData, 2, 0, ratio_w, ratio_h);
        drawLine(predCtx, predData, 1, 3, ratio_w, ratio_h);
        drawLine(predCtx, predData, 2, 4, ratio_w, ratio_h);
        drawLine(predCtx, predData, 3, 5, ratio_w, ratio_h);
        drawLine(predCtx, predData, 4, 6, ratio_w, ratio_h);
        drawLine(predCtx, predData, 5, 6, ratio_w, ratio_h);
        drawLine(predCtx, predData, 5, 7, ratio_w, ratio_h);
        drawLine(predCtx, predData, 6, 8, ratio_w, ratio_h);
        drawLine(predCtx, predData, 7, 9, ratio_w, ratio_h);
        drawLine(predCtx, predData, 8, 10, ratio_w, ratio_h);
        drawLine(predCtx, predData, 5, 11, ratio_w, ratio_h);
        drawLine(predCtx, predData, 6, 12, ratio_w, ratio_h);
        drawLine(predCtx, predData, 11, 12, ratio_w, ratio_h);
        drawLine(predCtx, predData, 11, 13, ratio_w, ratio_h);
        drawLine(predCtx, predData, 12, 14, ratio_w, ratio_h);
        drawLine(predCtx, predData, 13, 15, ratio_w, ratio_h);
        drawLine(predCtx, predData, 14, 16, ratio_w, ratio_h);

        console.log('ratio_w ' + ratio_w);
        console.log('ratio_h ' + ratio_h);


        // Draw keypoint and display inference info
        for (i = 0; i < len; i++) {
          drawKeyPoint(predCtx, predData[i], ratio_w, ratio_h);
        }

        // Measure prcess time
        measureProcessingTime(predCtx, nowTime);
      }

      // Calculate & Display DRP process time 
      drpData = 'DRP time:' + '\t\t' + datas.Value.drp_time + ' ms\n' +
        'Post-process time:' + '\t' + datas.Value.post_time + ' ms\n';

      predWindowData.value = predDatas;
      drpWindowData.value = drpData;
    }
    // ResNet50
    else if (datas.command_name === 'classfication_detection') {
      predCtx.linewidth = 8;
      predCtx.strokeStyle = 'red';
      predCtx.fillStyle = 'red';
      defaultCtx.fillStyle = 'red';

      predData = datas.Value.predict;
      len = predData.length;

      let cls = [];
      let pred = [];

      for (i = 0; i < len; i++) {
        predStr = predData[i];
        cls[i] = String(predStr.names);
        pred[i] = Number.parseFloat(predStr.pred).toFixed(2);

        if (i !== 0) {
          predDatas[i] = '\n' + cls[i] + ' :\t' + pred[i] + ' %';
        }
        else {
          predDatas[i] = cls[i] + ' :\t' + pred[i] + ' %';
        }
      }

      webcam.src = 'data:image/jpeg;base64,' + datas.Value.img;
      webcam.onload = function () {
        // Dispaly USB camera image
        predCtx.drawImage(webcam, 0, 0, 640, 480);

        // Display inference result
        for (i = 0; i < len; i++) {
          predCtx.fillText(cls[i], 5, 20 + (20 * i));
        }

        // Calculate process time
        measureProcessingTime(predCtx, nowTime);
      }

      // Calculate & Display DRP process time 
      drpData = 'DRP time:' + '\t\t' + datas.Value.drp_time + ' ms\n' +
        'Post-process time:' + '\t' + datas.Value.post_time + ' ms\n';

      predWindowData.value = predDatas;
      drpWindowData.value = drpData;
    }
  }
})
