package main

import (
	"encoding/binary"
	"log"
	"math"
	"time"

	"github.com/SMerrony/tello"
	"golang.org/x/mobile/app"
	"golang.org/x/mobile/event/lifecycle"
	"golang.org/x/mobile/event/paint"
	"golang.org/x/mobile/event/size"
	"golang.org/x/mobile/event/touch"
	"golang.org/x/mobile/exp/f32"
	"golang.org/x/mobile/exp/gl/glutil"
	"golang.org/x/mobile/exp/sensor"
	"golang.org/x/mobile/gl"
)

const vertShader = `#version 100

attribute vec4 position;

void main() {
	gl_Position = position;
}`

const fragShader = `#version 100

uniform mediump float grayLevel;

void main() {
	gl_FragColor = vec4(vec3(grayLevel), 1.0);
}`

type coord struct{ x, y, z float64 }

var (
	appSize size.Event

	velocity    coord
	yawVelocity float64
	flightData  tello.FlightData
	drone       tello.Tello

	glctx     gl.Context
	program   gl.Program
	position  gl.Attrib
	grayLevel gl.Uniform
	vertBuf   gl.Buffer
	lineVerts = f32.Bytes(binary.LittleEndian,
		-0.5, 0.0, 0.0,
		0.5, 0.0, 0.0,
		0.0, -0.5, 0.0,
		0.0, 0.5, 0.0,
	)
)

func main() {
	initDrone()
	app.Main(appMain)
}

func appMain(ap app.App) {
	sensor.Notify(ap)
	for evt := range ap.Events() {
		switch evt := ap.Filter(evt).(type) {
		case lifecycle.Event:
			switch evt.Crosses(lifecycle.StageVisible) {
			case lifecycle.CrossOn:
				glctx = evt.DrawContext.(gl.Context)
				onStart()
				ap.Send(paint.Event{})
			case lifecycle.CrossOff:
				onStop()
				glctx = nil
			}
		case paint.Event:
			if glctx == nil || evt.External {
				continue
			}
			onPaint()
			ap.Publish()
			ap.Send(paint.Event{})
		case size.Event:
			appSize = evt
		case touch.Event:
			onTouch(evt)
		case sensor.Event:
			onSensor(evt)
		}
	}
}

func onTouch(evt touch.Event) {
	if evt.Type == touch.TypeEnd || appSize.WidthPx < 2 || appSize.HeightPx < 2 {
		velocity.z, yawVelocity = 0.0, 0.0
	} else {
		yawVelocity = 2.0*float64(evt.X)/float64(appSize.WidthPx-1) - 1.0
		velocity.z = -(2.0*float64(evt.Y)/float64(appSize.HeightPx-1) - 1.0)
	}
	updateCtrl()
}

func onSensor(evt sensor.Event) {
	if evt.Sensor != sensor.Accelerometer {
		return
	}
	accel := coord{evt.Data[0], evt.Data[1], evt.Data[2]}
	if math.Abs(accel.z/9.8-1.0) > 3.0 {
		takeoffLand()
		return
	}

	roll, pitch := 0.0, 0.0
	if hyp := math.Sqrt(accel.x*accel.x + accel.z*accel.z); hyp != 0.0 {
		roll = math.Atan(accel.y / hyp)
	}
	if hyp := math.Sqrt(accel.y*accel.y + accel.z*accel.z); hyp != 0.0 {
		pitch = math.Atan(accel.x / hyp)
	}
	velocity.x = roll / (0.5 * math.Pi)
	velocity.y = -pitch / (0.5 * math.Pi)
	updateCtrl()

	flightData = drone.GetFlightData()
}

func onStart() {
	var err error
	if err = sensor.Enable(sensor.Accelerometer, 20*time.Millisecond); err != nil {
		log.Print(err)
	}
	if err = drone.ControlConnectDefault(); err != nil {
		log.Print(err)
	}
	program, err = glutil.CreateProgram(glctx, vertShader, fragShader)
	if err != nil {
		log.Print(err)
		return
	}
	vertBuf = glctx.CreateBuffer()
	glctx.BindBuffer(gl.ARRAY_BUFFER, vertBuf)
	glctx.BufferData(gl.ARRAY_BUFFER, lineVerts, gl.STATIC_DRAW)
	position = glctx.GetAttribLocation(program, "position")
	grayLevel = glctx.GetUniformLocation(program, "grayLevel")
}

func onStop() {
	resetCtrl()
	drone.ControlDisconnect()
	if err := sensor.Disable(sensor.Accelerometer); err != nil {
		log.Println(err)
	}
	glctx.DeleteProgram(program)
	glctx.DeleteBuffer(vertBuf)
}

func onPaint() {
	if flightData.BatteryCritical {
		glctx.ClearColor(0.5, 0.0, 0.0, 0.0)
	} else if flightData.BatteryLow {
		glctx.ClearColor(0.5, 0.5, 0.0, 0.0)
	} else if flightData.Flying {
		glctx.ClearColor(0.0, 0.25, 0.0, 0.0)
	} else {
		glctx.ClearColor(0.0, 0.0, 0.25, 0.0)
	}
	glctx.Clear(gl.COLOR_BUFFER_BIT)
	glctx.UseProgram(program)
	glctx.BindBuffer(gl.ARRAY_BUFFER, vertBuf)
	glctx.EnableVertexAttribArray(position)
	glctx.VertexAttribPointer(position, 3, gl.FLOAT, false, 0, 0)
	glctx.LineWidth(3)
	glctx.Uniform1f(grayLevel, 0.0)
	glctx.DrawArrays(gl.LINES, 0, 4)
	glctx.LineWidth(1)
	glctx.Uniform1f(grayLevel, 1.0)
	glctx.DrawArrays(gl.LINES, 0, 4)
	glctx.DisableVertexAttribArray(position)
}

func initDrone() {
	if err := drone.ControlConnectDefault(); err != nil {
		log.Println(err)
	}
}

func resetCtrl() {
	velocity, yawVelocity = coord{}, 0.0
	drone.Hover()
}

func takeoffLand() {
	resetCtrl()
	if flightData.Flying {
		drone.Land()
	} else {
		drone.TakeOff()
	}
}

func telloParam(val float64) int16 {
	return int16(math.MaxInt16 * math.Max(-1.0, (math.Min(1.0, val))))
}

func updateCtrl() {
	drone.UpdateSticks(tello.StickMessage{
		Rx: telloParam(velocity.x),
		Ry: telloParam(velocity.y),
		Lx: telloParam(yawVelocity),
		Ly: telloParam(velocity.z),
	})
}
