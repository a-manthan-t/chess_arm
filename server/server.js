import crypto from "crypto"
import express from "express"
import http from "http"
import path from "path"
import { fileURLToPath } from "url"
import { v4 as uuid } from "uuid"
import { WebSocketServer } from "ws"

const resources = `${path.dirname(fileURLToPath(import.meta.url))}/res`
const hashConfig = {
    salt: crypto.randomBytes(16),
    iterations: 100_000,
    keylen: 32,
    digest: "sha512"
}

const app = express()
const server = http.createServer(app)
const wss = new WebSocketServer({ server })

function broadcast(message) {
    wss.clients.forEach(client => {
        if (!client.isRobot) {
            client.send(message)
        }
    })
}

const streamingHash = crypto.pbkdf2Sync(
    process.env.STREAMING_TOKEN,
    hashConfig.salt,
    hashConfig.iterations,
    hashConfig.keylen,
    hashConfig.digest
);

// Add viewer hash

app.use(express.static("res"))
app.get("/", (req, res) => res.sendFile(`${resources}/index.html`))
wss.on("connection", ws => {
    ws.isRobot = false

    ws.onmessage = msg => {
        if (typeof msg.data === "string") {
            const query = msg.data.split(";")

            switch (query[0]) {
                case "promote":
                    if (!ws.isRobot && query[1] !== undefined) {
                        const userHash = crypto.pbkdf2Sync(
                            query[1],
                            hashConfig.salt,
                            hashConfig.iterations,
                            hashConfig.keylen,
                            hashConfig.digest
                        )

                        if (crypto.timingSafeEqual(streamingHash, userHash)) {
                            ws.isRobot = true
                            ws.uuid = uuid()
                            broadcast(`new_robot;ws.uuid`)
                            console.log(`New Robot: ${ws.uuid}`)
                            ws.send("success")
                            break
                        }
                    }

                    ws.send("fail")
                    break
                case "cmd":
                    if (!ws.isRobot && query[1] !== undefined) {

                    }

                    ws.send("fail")
                    break
            }
        } else if (ws.isRobot && msg.data instanceof Buffer) {
            broadcast(`frame;${ws.uuid}`)
            broadcast(msg.data) // maybe combine into one (via blobs)?
        }
    }
})

server.listen(8008, () => console.log("Running on port 8008..."))
