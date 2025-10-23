package main

import (
	"math"
	_ "math/rand"

	rl "github.com/gen2brain/raylib-go/raylib"
)

// -------------------------------
// Configurable Parameters
// -------------------------------
const (
	particleCount = 1000
	restDensity   = 1000.0
	gasConstant   = 50.0
	viscosity     = 250.0
	h             = 16.0 // smoothing radius
	mass          = 200.0
	timeStep      = 0.0015 // seconds per update
	gravity       = 3000.0
	windowWidth   = 800
	windowHeight  = 400
)

// -------------------------------
// Data Structures
// -------------------------------
type Particle struct {
	pos, vel          rl.Vector2
	density, pressure float64
}

type SPHSim struct {
	particles []Particle
	grid      Grid
}

type Grid struct {
	cellSize float32
	cells    map[[2]int][]int
}

// -------------------------------
// Kernel Functions
// -------------------------------
func poly6(r, h float64) float64 {
	if r >= 0 && r <= h {
		return (315.0 / (64.0 * math.Pi * math.Pow(h, 9))) *
			math.Pow(h*h-r*r, 3)
	}
	return 0
}

func spikyGrad(rij rl.Vector2, r, h float64) rl.Vector2 {
	if r > 0 && r <= h {
		m := -45.0 / (math.Pi * math.Pow(h, 6)) * math.Pow(h-r, 2)
		return rl.Vector2Scale(rij, float32(m/r))
	}
	return rl.Vector2{}
}

func viscLaplacian(r, h float64) float64 {
	if r >= 0 && r <= h {
		return 45.0 / (math.Pi * math.Pow(h, 6)) * (h - r)
	}
	return 0
}

// -------------------------------
// Grid Calculations
// -------------------------------

func (g *Grid) Clear() {
	for k := range g.cells {
		g.cells[k] = g.cells[k][:0]
	}
}
func (g *Grid) Insert(particles []Particle) {
	g.Clear()
	for i, p := range particles {
		key := [2]int{int(p.pos.X / g.cellSize), int(p.pos.Y / g.cellSize)}
		g.cells[key] = append(g.cells[key], i)
	}
}

func (g *Grid) Nearby(p Particle) []int {
	key := [2]int{int(p.pos.X / g.cellSize), int(p.pos.Y / g.cellSize)}
	var ids []int
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			k := [2]int{key[0] + dx, key[1] + dy}
			ids = append(ids, g.cells[k]...)
		}
	}
	return ids
}

// -------------------------------
// SPH Core
// -------------------------------
func (s *SPHSim) computeDensities() {
	for i := range s.particles {
		pi := &s.particles[i]
		pi.density = 0
		for _, j := range s.grid.Nearby(*pi) {
			pj := &s.particles[j]
			rv := rl.Vector2Subtract(pi.pos, pj.pos)
			r := rl.Vector2Length(rv)
			if r < float32(h) {
				pi.density += mass * poly6(float64(r), h)
			}
		}
		pi.pressure = gasConstant * (pi.density - restDensity)
	}
}

func (s *SPHSim) computeForces() {
	for i := range s.particles {
		pi := &s.particles[i]
		acc := rl.Vector2{X: 0, Y: gravity}
		for _, j := range s.grid.Nearby(*pi) {
			if i == j {
				continue
			}
			pj := &s.particles[j]
			rij := rl.Vector2Subtract(pi.pos, pj.pos)
			r := rl.Vector2Length(rij)
			if r <= 0 || r > float32(h) {
				continue
			}
			// Pressure
			pressureTerm := -mass * (pi.pressure + pj.pressure) / (2 * pj.density)
			grad := spikyGrad(rij, float64(r), h)
			acc = rl.Vector2Add(acc, rl.Vector2Scale(grad, float32(pressureTerm/pj.density)))
			// Viscosity
			dv := rl.Vector2Subtract(pj.vel, pi.vel)
			visc := viscosity * viscLaplacian(float64(r), h)
			acc = rl.Vector2Add(acc, rl.Vector2Scale(dv, float32(visc/pj.density)))
		}
		// Integrate acceleration
		pi.vel = rl.Vector2Add(pi.vel, rl.Vector2Scale(acc, timeStep))
	}
}

func (s *SPHSim) integrate() {
	for i := range s.particles {
		p := &s.particles[i]
		p.pos = rl.Vector2Add(p.pos, rl.Vector2Scale(p.vel, timeStep))
		// simple wall collisions
		if p.pos.X < 5 {
			p.pos.X = 5
			p.vel.X *= -0.5
		}
		if p.pos.X > float32(windowWidth-5) {
			p.pos.X = float32(windowWidth - 5)
			p.vel.X *= -0.5
		}
		if p.pos.Y < 5 {
			p.pos.Y = 5
			p.vel.Y *= -0.5
		}
		if p.pos.Y > float32(windowHeight-5) {
			p.pos.Y = float32(windowHeight - 5)
			p.vel.Y *= -0.5
		}
		// clamp velocity
		speed := rl.Vector2Length(p.vel)
		if speed > 1000 {
			p.vel = rl.Vector2Scale(rl.Vector2Normalize(p.vel), 1000)
		}

		drag := float32(0.995)
		p.vel = rl.Vector2Scale(p.vel, drag)
	}
}

func (s *SPHSim) TotalKineticEnergy() float64 {
	var total float64
	for _, p := range s.particles {
		v := rl.Vector2Length(p.vel)
		total += 0.5 * mass * float64(v*v)
	}
	return total
}

func (s *SPHSim) Step() {
	s.grid.Insert(s.particles)
	s.computeDensities()
	s.computeForces()
	s.integrate()
}

// -------------------------------
// Initialization
// -------------------------------
func NewSPHSim() *SPHSim {
	s := &SPHSim{}
	s.particles = make([]Particle, particleCount)
	s.grid = Grid{cellSize: float32(h), cells: make(map[[2]int][]int)}
	// for i := range s.particles {
	// 	x := float32(300 + rand.Float32()*100)
	// 	y := float32(rand.Float32()*50 + 50)
	// 	s.particles[i] = Particle{
	// 		pos: rl.Vector2{X: x, Y: y},
	// 		vel: rl.Vector2{X: rand.Float32()*50 - 25, Y: 0},
	// 	}
	// }

	cols := int(math.Sqrt(particleCount))
	rows := cols
	spacing := float32(10)
	for y := 0; y < rows; y++ {
		for x := 0; x < cols; x++ {
			i := y*cols + x
			if i >= len(s.particles) {
				break
			}
			s.particles[i].pos = rl.Vector2{
				X: 200 + float32(x)*spacing,
				Y: 50 + float32(y)*spacing,
			}
		}
	}
	return s
}

// -------------------------------
// Main
// -------------------------------
func main() {
	rl.InitWindow(windowWidth, windowHeight, "Minimal 2D SPH Prototype")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	sim := NewSPHSim()
	energyHistory := make([]float64, 0, 1000)
	for !rl.WindowShouldClose() {
		// Simulation step: small fixed timestep for stability
		steps := 5 // substeps for smoother integration
		for i := 0; i < steps; i++ {
			sim.Step()
		}
		energy := sim.TotalKineticEnergy()
		energyHistory = append(energyHistory, energy)
		if len(energyHistory) > windowWidth {
			energyHistory = energyHistory[1:]
		}

		//---------------------------------
		// Render
		//---------------------------------
		rl.BeginDrawing()
		rl.ClearBackground(rl.Black)
		for _, p := range sim.particles {
			c := uint8(math.Min((p.density/restDensity)*255, 255))
			rl.DrawCircle(int32(p.pos.X), int32(p.pos.Y), 3,
				rl.NewColor(c, 100, 255-c/2, 255))
		}
		if len(energyHistory) > 1 {
			maxE := 0.0
			for _, e := range energyHistory {
				if e > maxE {
					maxE = e
				}
			}
			if maxE == 0 {
				maxE = 1.0
			}

			graphHeight := float32(100)
			baseY := float32(graphHeight)

			for i := 1; i < len(energyHistory); i++ {
				x1 := float32(i - 1)
				x2 := float32(i)
				y1 := baseY - (float32(energyHistory[i-1]/maxE) * (graphHeight - 5))
				y2 := baseY - (float32(energyHistory[i]/maxE) * (graphHeight - 5))
				rl.DrawLine(int32(x1), int32(y1), int32(x2), int32(y2), rl.NewColor(0, 255, 0, 255))
			}
			rl.DrawText("Kinetic Energy", 5, 5, 10, rl.Green)
		}
		rl.EndDrawing()
	}
}
