package main

import (
	"encoding/binary"
	"log"
	"math"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
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

void main() {
	gl_FragColor = vec4(1.0);
}`

type coord struct{ x, y, z float64 }

type droneCtrlFunc func(val int) error

var (
	appSize         size.Event
	velocity        coord
	yawVelocity     float64
	flying          bool
	takeoffLandTime time.Time
	drone           *tello.Driver

	glctx     gl.Context
	program   gl.Program
	position  gl.Attrib
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
				if err := sensor.Enable(sensor.Accelerometer, 20*time.Millisecond); err != nil {
					log.Println(err)
				}
				glctx = evt.DrawContext.(gl.Context)
				onStart()
				ap.Send(paint.Event{})
			case lifecycle.CrossOff:
				resetCtrl()
				if err := sensor.Disable(sensor.Accelerometer); err != nil {
					log.Println(err)
				}
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
	if evt.Type == touch.TypeEnd {
		velocity.z, yawVelocity = 0.0, 0.0
		yawVelocity = 0.0
	} else {
		yawVelocity = 200.0*float64(evt.X)/float64(appSize.WidthPx-1) - 100.0
		velocity.z = 200.0*float64(evt.Y)/float64(appSize.HeightPx-1) - 100.0
	}
	if flying {
		updateCtrl()
	}
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
	velocity.x = 100.0 * roll / (0.5 * math.Pi)
	velocity.y = 100.0 * pitch / (0.5 * math.Pi)
	if flying {
		updateCtrl()
	}
}

func onStart() {
	var err error
	program, err = glutil.CreateProgram(glctx, vertShader, fragShader)
	if err != nil {
		log.Print(err)
		return
	}

	vertBuf = glctx.CreateBuffer()
	glctx.BindBuffer(gl.ARRAY_BUFFER, vertBuf)
	glctx.BufferData(gl.ARRAY_BUFFER, lineVerts, gl.STATIC_DRAW)
	position = glctx.GetAttribLocation(program, "position")
}

func onStop() {
	glctx.DeleteProgram(program)
	glctx.DeleteBuffer(vertBuf)
}

func onPaint() {
	if flying {
		glctx.ClearColor(0.0, 0.25, 0.0, 0.0)
	} else {
		glctx.ClearColor(0.25, 0.0, 0.0, 0.0)
	}
	glctx.Clear(gl.COLOR_BUFFER_BIT)
	glctx.UseProgram(program)
	glctx.LineWidth(3)
	glctx.BindBuffer(gl.ARRAY_BUFFER, vertBuf)
	glctx.EnableVertexAttribArray(position)
	glctx.VertexAttribPointer(position, 3, gl.FLOAT, false, 0, 0)
	glctx.DrawArrays(gl.LINES, 0, 4)
	glctx.DisableVertexAttribArray(position)
}

func initDrone() {
	drone = tello.NewDriver("8888")
	robot := gobot.NewRobot("tello", []gobot.Connection{}, []gobot.Device{drone})
	robot.Start(false)
}

func resetCtrl() {
	velocity, yawVelocity = coord{}, 0.0
	updateCtrl()
}

func takeoffLand() {
	if time.Since(takeoffLandTime) < 1*time.Second {
		return
	}

	resetCtrl()
	if flying {
		drone.Land()
	} else {
		drone.TakeOff()
	}
	flying = !flying
	takeoffLandTime = time.Now()
}

func updateCtrl() {
	droneCtrl(drone.Right, drone.Left, velocity.x)
	droneCtrl(drone.Backward, drone.Forward, velocity.y)
	droneCtrl(drone.Down, drone.Up, velocity.z)
	droneCtrl(drone.Clockwise, drone.CounterClockwise, yawVelocity)
}

func droneCtrl(posCtrl, negCtrl droneCtrlFunc, val float64) {
	arg := int(math.Max(0.0, math.Min(100.0, math.Abs(val))))
	if val >= 0.0 {
		posCtrl(arg)
	} else {
		negCtrl(arg)
	}
}
