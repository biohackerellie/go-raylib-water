package main

import (
	"math"

	rl "github.com/gen2brain/raylib-go/raylib"
)

/*
* Droplets
 */

type Droplet struct {
	volume     float64 // How much water this cell contains (0.0 to 1.0)
	size       int
	isObstacle bool // Is this cell an obstacle?

	vx, vy   float64 // Velocity components
	pressure float64 // hydrostatic pressure
}

func (d *Droplet) Draw(x, y, tileSize int, hasWaterAbove bool) {
	// Convert grid coordinates to pixel coordinates
	pixelX := x * tileSize
	pixelY := y * tileSize

	if d.isObstacle {
		// Draw obstacle as brown rectangle
		rl.DrawRectangle(int32(pixelX), int32(pixelY), int32(tileSize), int32(tileSize), rl.Brown)
	}

	if d.volume > 0 {
		// Calculate visual height based on volume
		// Full volume (1.0) = full tile height, half volume (0.5) = half tile height
		height := int(float64(tileSize) * d.volume)

		// Fill up from the bottom
		offsetY := tileSize - height
		// if water above, fill from the top
		if hasWaterAbove {
			offsetY = 0
		}
		// Draw the droplet
		pressureColor := uint8(math.Min(d.pressure*40+d.volume*100, 255))
		rl.DrawRectangle(int32(pixelX), int32(pixelY+offsetY), int32(tileSize), int32(tileSize), rl.NewColor(0, 0, pressureColor, 255))
	}
}

func CreateWaterGenerator(x, y, tileSize int, state *[][]Droplet) {
	for xOffset := 0; xOffset <= 4; xOffset++ {
		droplet := Droplet{size: tileSize, volume: 1.0}
		(*state)[y][x+xOffset] = droplet
	}
}

func CreateHorizontalObstacle(x, y, size int, state *[][]Droplet) {
	for offset := 0; offset < size; offset++ {
		(*state)[y][x+offset].isObstacle = true
		(*state)[y+1][x+offset].isObstacle = true
		(*state)[y+2][x+offset].isObstacle = true
	}
}
func CreateVerticalObstacle(x, y, size int, state *[][]Droplet) {
	for offset := 0; offset < size; offset++ {
		(*state)[y+offset][x].isObstacle = true
		(*state)[y+offset][x+1].isObstacle = true
		(*state)[y+offset][x+2].isObstacle = true
	}
}

/*
* Game / GameState
 */

type Game struct {
	Width    int
	Height   int
	State    [][]Droplet // 2D grid of droplets
	tileSize int
}

func NewGame(w, h, ts int) *Game {

	g := &Game{Width: w, Height: h, tileSize: ts}

	// Create the new game state
	// divide pixel dimensions by tile size to get grid size
	g.State = CreateGameState(g.Width/g.tileSize, g.Height/g.tileSize, ts)
	return g
}

func (g *Game) Draw() {
	// Loop through the grid and draw each droplet
	for y := range g.State {
		for x := 0; x < len(g.State[y]); x++ {
			// Check if there is water above this cell
			hasWaterAbove := y > 0 && g.State[y-1][x].volume > 0
			g.State[y][x].Draw(x, y, g.tileSize, hasWaterAbove)
		}
	}
}

func CreateGameState(w, h, ts int) [][]Droplet {
	// Create the new game state
	newState := make([][]Droplet, h)
	// Loop through each row of the grid
	for y := range h {
		// Create the columns
		newState[y] = make([]Droplet, w)

		// Loop through each cell and create a new droplet
		for x := range newState[y] {
			newState[y][x] = Droplet{
				size: ts,
			}
		}
	}
	return newState
}

func (g *Game) Update() {
	// Create a new state to avoid modifying the current one
	newState := CreateGameState(len(g.State[0]), len(g.State), g.tileSize)

	// Copy current state to new state
	for y := range g.State {
		copy(newState[y], g.State[y])
	}

	computePressures(&newState)
	dampenPressure(&newState)
	for y := len(g.State) - 1; y >= 0; y-- {
		for x := range g.State[y] {

			if g.State[y][x].isObstacle {
				newState[y][x] = g.State[y][x]
				continue
			}
			// Only process cells that contain water
			if g.State[y][x].volume > 0 {
				// Check if we are at the bottom
				if y+1 < len(g.State) {
					processWaterCell(x, y, &newState)

				}
			}
		}
	}

	// Replace old state with new calculated state
	g.State = newState
}

func processWaterCell(x, y int, newState *[][]Droplet) {
	// Try to flow downards, as if by gravity(but not into obstacles)
	if y+1 < len(*newState) && !(*newState)[y+1][x].isObstacle {
		fill(&(*newState)[y][x], &(*newState)[y+1][x], 1.0, 0.5)
	}

	// If water can still flow down, don't try other directions yet
	if canFlowDown(x, y, newState) {
		return
	}

	// Water spreads sideways when blocked below
	tryHorizontalFlow(x, y, newState)

	if (*newState)[y][x].volume > 0 {
		tryDiagonalFlow(x, y, newState)
	}

	applyPressureFlow(x, y, newState)
}

func applyPressureFlow(x, y int, newState *[][]Droplet) {
	current := &(*newState)[y][x]
	if current.isObstacle || current.volume <= 0 {
		return
	}

	// Directions: up, down, left, right
	directions := [][2]int{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}
	for _, dpos := range directions {
		nx, ny := x+dpos[0], y+dpos[1]
		if ny < 0 || ny >= len(*newState) || nx < 0 || nx >= len((*newState)[0]) {
			continue
		}
		neighbor := &(*newState)[ny][nx]
		if neighbor.isObstacle {
			continue
		}
		if dpos[1] == -1 && (*newState)[y-1][x].isObstacle {
			continue
		}

		// Compute combined pressure difference
		pressureDiff := (current.pressure + current.volume) - (neighbor.pressure + neighbor.volume)

		dy := float64(y - ny)

		if pressureDiff > 0 {
			flow := 0.05 * pressureDiff
			if flow > current.volume {
				flow = current.volume
			}
			if dy == 0 {
				flow *= 0.7
			}
			if pressureDiff >= 0.0001 {
				continue
			}
			current.volume -= flow
			neighbor.volume += flow
			current.volume = math.Min(1.0, math.Max(0.0, current.volume))
			neighbor.volume = math.Min(1.0, math.Max(0.0, neighbor.volume))
			dvx := float64(nx - x)
			dvy := float64(ny - y)
			current.vx += dvx * flow * 0.1
			current.vy += dvy * flow * 0.1
		}
	}
}
func dampenPressure(state *[][]Droplet) {
	for y := range *state {
		for x := range (*state)[y] {
			d := &(*state)[y][x]
			d.vx *= 0.9
			d.vy *= 0.9
			d.pressure *= 1.1
		}
	}
}
func computePressures(state *[][]Droplet) {
	for y := len(*state) - 1; y >= 0; y-- {
		for x := range (*state)[y] {
			d := &(*state)[y][x]
			if d.isObstacle {
				d.pressure = 0
				continue
			}
			if y == len(*state)-1 || (*state)[y+1][x].isObstacle {
				// bottom row
				d.pressure = d.volume
			} else {
				d.pressure = (*state)[y+1][x].pressure + d.volume
			}
		}
	}
}
func canFlowDown(x, y int, state *[][]Droplet) bool {
	return y+1 < len(*state) && (*state)[y+1][x].volume < 1.0 && !(*state)[y+1][x].isObstacle
}

func tryHorizontalFlow(x, y int, state *[][]Droplet) {
	current := &(*state)[y][x]

	// Only cascade if there's water below
	hasWaterBelow := y+1 < len(*state) && (*state)[y+1][x].volume > 0.5
	if !hasWaterBelow {
		return
	}

	// Cascade right - distribute to multiple cells
	for offset := 1; offset <= 3 && x+offset < len((*state)[y]); offset++ {
		target := &(*state)[y][x+offset]
		if target.volume < current.volume && !target.isObstacle {
			flowRate := (current.volume - target.volume) * 0.1 / float64(offset)
			fill(current, target, 1.0, flowRate)
		}
	}

	// Cascade left - distribute to multiple cells
	for offset := 1; offset <= 3 && x-offset >= 0; offset++ {
		target := &(*state)[y][x-offset]
		if target.volume < current.volume && !target.isObstacle {
			flowRate := (current.volume - target.volume) * 0.1 / float64(offset)
			fill(current, target, 1.0, flowRate)
		}
	}
}

func tryDiagonalFlow(x, y int, state *[][]Droplet) {
	current := &(*state)[y][x]

	// Flow diagonally down-right if space is available
	if x+1 < len((*state)[y]) && y+1 < len(*state) && (*state)[y+1][x+1].volume < 1.0 && !(*state)[y+1][x+1].isObstacle {
		fill(current, &(*state)[y+1][x+1], 1.0, 0.25)
	}

	// Flow diagonally down-left if space is available
	if x-1 > 0 && y+1 < len(*state) && (*state)[y+1][x-1].volume < 1.0 && !(*state)[y+1][x-1].isObstacle {
		fill(current, &(*state)[y+1][x-1], 1.0, 0.25)
	}

}

// Calculate how much more water a droplet can hold
func remainder(droplet Droplet, maxVolume float64) float64 {
	return maxVolume - droplet.volume
}

// Fill transfers water between two droplets at a controlled rate
func fill(current, target *Droplet, maxVolume, flowRate float64) {

	// Calculate how much water can be transferred
	transfer := remainder(*target, maxVolume)

	// Limit transfer to the flow rate (prevents instant teleportation)
	if transfer > flowRate {
		transfer = flowRate
	}

	// Move water from source to target
	current.volume -= transfer
	target.volume += transfer
}

/*
* Main
 */

func main() {
	// Create a new game
	var game = NewGame(1920, 1080, 20)
	// Initialize Raylib
	rl.InitWindow(int32(game.Width), int32(game.Height), "WaterSim")
	defer rl.CloseWindow()

	// Set up a counter, so we can spawn new water at a rate
	frameCount := 0
	flowStartX := 400 / game.tileSize
	flowStartY := 10 / game.tileSize

	CreateWaterGenerator(flowStartX, flowStartY, game.tileSize, &game.State)
	CreateVerticalObstacle(10, 10, 20, &game.State)
	CreateHorizontalObstacle(10, 30, 50, &game.State)
	CreateHorizontalObstacle(40, 20, 40, &game.State)
	gridWidth := len(game.State[0])
	gridHeight := len(game.State)

	// Top border
	CreateHorizontalObstacle(0, 0, gridWidth, &game.State)
	for x := flowStartX; x < flowStartX+5; x++ {
		game.State[0][x].isObstacle = false
		game.State[1][x].isObstacle = false
		game.State[2][x].isObstacle = false
	}

	// Bottom border (y = last few rows)
	CreateHorizontalObstacle(0, gridHeight-3, gridWidth, &game.State)

	// Left border
	CreateVerticalObstacle(0, 0, gridHeight, &game.State)

	// Right border (x = last few columns)
	CreateVerticalObstacle(gridWidth-3, 0, gridHeight, &game.State)

	// Set the target frame rate
	rl.SetTargetFPS(60)

	// Main game loop
	for !rl.WindowShouldClose() {
		frameCount++
		// Begin drawing
		rl.BeginDrawing()
		rl.ClearBackground(rl.Black)

		// Add new water every 5 frames (creates a continuous water stream)
		if frameCount%5 == 0 {
			for x := 0; x < 5; x++ {
				cell := &game.State[flowStartY][flowStartX+x]
				if !cell.isObstacle && cell.volume < 0.7 {
					cell.volume = 1.0
				}
			}
			// CreateWaterGenerator(flowStartX, flowStartY, game.tileSize, &game.State)
		}

		// Draw the game
		game.Draw()

		// Update the game state based on the rules
		game.Update()

		rl.EndDrawing()
	}
}
