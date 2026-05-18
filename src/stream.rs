use std::net::{TcpListener, TcpStream};

use opencv::{
    core::{Mat, Vector},
    imgcodecs::imencode_def,
};
use tungstenite::{Bytes, Message, Utf8Bytes, WebSocket, accept};

const PROMOTE: Message = Message::Text(Utf8Bytes::from_static("promote"));
pub const CAMERA_ERROR: Message = Message::Text(Utf8Bytes::from_static("camera"));
pub const GAME_OVER: Message = Message::Text(Utf8Bytes::from_static("over"));
pub const INVALID: Message = Message::Text(Utf8Bytes::from_static("invalid"));

/// Starts a stream for the robot's perspective and waits for a connection.
pub fn start_stream() -> WebSocket<TcpStream> {
    let server = TcpListener::bind("0.0.0.0:8080").expect("Could not start stream.");
    let (stream, _) = server.accept().expect("Error whilst connecting.");
    accept(stream).expect("Could not establish Websocket connection.")
}

/// Sends an image to the connected client.
pub fn send_image(websocket: &mut WebSocket<TcpStream>, image: &Mat) {
    let mut bytes = Vector::new();
    imencode_def(".jpg", image, &mut bytes).ok();
    websocket
        .send(Message::Binary(Bytes::from_owner(bytes)))
        .ok();
}

/// Inform the client that intervention with the robot is required
/// and blocks till the client notifies the server.
pub fn wait_on_notification(websocket: &mut WebSocket<TcpStream>) {
    websocket.send(PROMOTE).ok();
    websocket.read().ok();
}

#[cfg(test)]
mod tests {
    use std::{
        thread::{self, sleep},
        time::Duration,
    };

    use opencv::{
        core::Vector,
        imgcodecs::{imencode_def, imread_def},
    };
    use tungstenite::{Message, Utf8Bytes, connect};

    use crate::stream::{send_image, start_stream, wait_on_notification};

    #[test]
    fn stream_test() {
        let image = imread_def("test_res/move0.jpg").unwrap();
        let mut jpg = Vector::new();
        imencode_def(".jpg", &image, &mut jpg).ok();

        let server_thread = thread::spawn(move || {
            let mut websocket = start_stream();
            send_image(&mut websocket, &image);
            wait_on_notification(&mut websocket);
        });

        sleep(Duration::from_millis(10));

        let mut client = connect("ws://localhost:8080/").unwrap().0;
        let received = client.read().unwrap().into_data().to_vec();
        assert_eq!(received.as_slice(), jpg.as_slice());

        client
            .send(Message::Text(Utf8Bytes::from_static("done")))
            .ok();
        server_thread.join().ok();
    }
}
