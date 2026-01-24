import crypto from "crypto"
import express from "express"
import http from "http"
import path from "path"
import { fileURLToPath } from "url"
import { v4 as uuid } from "uuid"
import { WebSocketServer } from "ws"

/* Server Construction */

const app = express()
const server = http.createServer(app)
const wss = new WebSocketServer({ server })

/* Utilities */

let robots = [] // Keep track of connected robots' ids.

function broadcast(message, filterFun = (_) => true, status = 2) {
    wss.clients.forEach(client => {
        if (client.status === status && filterFun(client)) {
            client.send(message)
        }
    })
}

function validateStop(command) {
    const coords = command.split(",")
    return coords.length === 3 && coords.every(element => !isNaN(parseFloat(element)));
}

/* Config */

const resources = `${path.dirname(fileURLToPath(import.meta.url))}/res`
const hashConfig = {
    salt: crypto.randomBytes(16),
    iterations: 100_000,
    keylen: 32,
    digest: "sha512"
}

const viewingHash = crypto.pbkdf2Sync(
    process.env.VIEWING_TOKEN,
    hashConfig.salt,
    hashConfig.iterations,
    hashConfig.keylen,
    hashConfig.digest
);

const streamingHash = crypto.pbkdf2Sync(
    process.env.STREAMING_TOKEN,
    hashConfig.salt,
    hashConfig.iterations,
    hashConfig.keylen,
    hashConfig.digest
);

/* Listeners */

app.use(express.static("res"))
app.get("/", (req, res) => res.sendFile(`${resources}/index.html`))

wss.on("connection", ws => {
    ws.status = 0 // 0 = new, 1 = robot, 2 = viewer

    ws.onmessage = msg => {
        if (typeof msg.data === "string") {
            const query = msg.data.split(";")
            let valid = true

            switch (query[0]) {
                case "promote": // Request by a robot to publish a stream.
                    if (ws.status === 0 && query[1] !== undefined) {
                        const userHash = crypto.pbkdf2Sync(
                            query[1],
                            hashConfig.salt,
                            hashConfig.iterations,
                            hashConfig.keylen,
                            hashConfig.digest
                        )

                        if (crypto.timingSafeEqual(streamingHash, userHash)) {
                            ws.status = 1
                            ws.uuid = uuid()
                            robots.push(ws.uuid)

                            wss.clients.forEach(client => {
                                if (client.currentRobot === "") {
                                    client.currentRobot = robots[0]
                                }
                            })

                            console.log(`New Robot: ${ws._socket.remoteAddress}, ${ws.uuid}`)
                            broadcast(`new_robot`)
                            ws.send("promotion_success")
                        } else {
                            ws.send("promotion_fail")
                        }
                    } else valid = false
                    break
                case "grant_viewing": // Request by a new user to view streams.
                    if (ws.status === 0 && query[1] !== undefined) {
                        const userHash = crypto.pbkdf2Sync(
                            query[1],
                            hashConfig.salt,
                            hashConfig.iterations,
                            hashConfig.keylen,
                            hashConfig.digest
                        )

                        if (crypto.timingSafeEqual(viewingHash, userHash)) {
                            ws.status = 2
                            ws.currentRobot = robots.length !== 0 ? robots[0] : "";

                            console.log(`New Viewer: ${ws._socket.remoteAddress}`)
                            ws.send(`viewing_granted;${robots.length}`)
                        } else {
                            ws.send("viewing_denied")
                        }
                    } else valid = false
                    break
                case "switch": // Change to a different stream.
                    if (ws.status === 2 && query[1] !== undefined && !isNaN(parseInt(query[1]))) {
                        ws.currentRobot = robots[parseInt(query[1])]
                    } else valid = false
                    break
                case "stop": // Emergency stop for the robot.
                    if (ws.status === 2 && (query[1] !== undefined && validateStop(query[1]) || query[2] === "true") && query[3] !== undefined) {
                        const auto = query[2] === "true" ? 1 : 0
                        const message = auto ? [0, 0, 0] : query[1].split(",").map(element => parseFloat(element))
                        const abort = query[3] === "true" ? 1 : 0

                        console.log(`Emergency ${abort ? "Abort" : "Pause"} Ordered on ${ws.currentRobot} to (${auto ? "Auto" : message.join(", ")})`)
                        broadcast(new Float32Array(message.concat([auto, abort])), (client) => ws.currentRobot === client.uuid, 1)
                    } else valid = false
                    break
                default: valid = false
            }

            if (!valid) {
                ws.send("invalid") // Requests made via the provided frontend/robot are always valid.
            }
        } else if (ws.status === 1 && msg.data instanceof Buffer) { // Byte buffers are received from the robots.
            // Only send to viewers who are currently watching the given robot.
            broadcast(msg.data, (client) => client.currentRobot === ws.uuid)
        }
    }

    ws.onclose = _ => {
        if (ws.status === 1) {
            robots = robots.filter(robot => robot !== ws.uuid) // Remove disconnected robot.

            // Reset viewers to first view (although this code doesn't care
            // about whether the viewers were actually viewing this robot).
            wss.clients.forEach(client => {
                if (client.status === 2) {
                    client.currentRobot = robots.length !== 0 ? robots[0] : "";
                }
            })

            console.log(`Robot Disconnected: ${ws._socket.remoteAddress}`)
            broadcast("robot_disconnected")
        }
    }
})

/* Start Server */

server.listen(8008, () => console.log("Running on port 8008..."))
