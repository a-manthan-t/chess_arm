const ws = new WebSocket(`ws://${window.location.host}`)

var currentRobot = 1
var robotCount = 0

function setRobotCount() {
    document.getElementById("robot_count").innerHTML
        = `${robotCount === 0 ? 0 : currentRobot} of ${robotCount}`
}

function switchRobot(left = true) {
    currentRobot += left ? -1 : 1

    // Wrap around.
    if (currentRobot === 0) {
        currentRobot = robotCount
    } else if (currentRobot === robotCount + 1) {
        currentRobot = 1
    }

    ws.send(`switch;${currentRobot - 1}`)
}

ws.onmessage = msg => {
    if (msg.data instanceof Blob) { // Happens when we get an image from the server.
        document.getElementById("view").src = URL.createObjectURL(msg.data)
    } else {
        let data = msg.data.split(";")

        switch (data[0]) {
            case "new_robot":
                robotCount++
                setRobotCount()
                break
            case "robot_disconnected":
                robotCount--
                currentRobot = 1
                setRobotCount()
                break
            case "viewing_granted":
                robotCount = parseInt(data[1])
                setRobotCount()
                document.getElementById("viewer").removeAttribute("hidden")
                break
            case "viewing_denied":
                document.getElementById("login_menu").removeAttribute("hidden")
                document.getElementById("login_label").innerHTML = "Incorrect Token, Try Again"
                break
        }
    }
}

// Authenticate to receive streams.
document.getElementById("send").onclick = () => {
    ws.send("grant_viewing;" + document.getElementById("token").value)
    document.getElementById("token").value = ""
    document.getElementById("login_label").innerHTML = "Enter Login Token"
    document.getElementById("login_menu").setAttribute("hidden", true)
}

document.getElementById("previous").onclick = switchRobot
document.getElementById("next").onclick = () => switchRobot(false)
