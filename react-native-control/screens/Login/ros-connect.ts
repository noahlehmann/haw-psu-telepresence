import {useEffect, useState} from 'react';
import {Ros, Topic} from 'roslib';
import {MovementDirection, MOVEMENTS} from './movements';

export const enum ConnectionStatus {
  CONNECTED,
  DISCONNECTED,
  CONNECTING,
  FAILED,
}

export const useRosConnect = () => {
  const [connection, setConnection] = useState<null | Ros>(null);
  const [publishTopic, setPublishTopic] = useState<null | Topic>(null);
  const [connectionState, setConnectionState] = useState(ConnectionStatus.DISCONNECTED);
  const [intervalId, setIntervalId] = useState(0);

  const createListener = () => {
    if (!connection) {
      throw new Error('Connection is not set');
    }
    connection.on('connection', () => {
      console.log('Connected to websocket server.');
      setConnectionState(ConnectionStatus.CONNECTED);
      createPublishTopic();
    });
    connection.on('error', () => {
      console.log('Error connecting to websocket server: ', connection);
      setConnectionState(ConnectionStatus.FAILED);
      setConnection(null);
      setPublishTopic(null);
    });
    connection.on('close', () => {
      console.log('Connection to websocket server closed.');
      setConnectionState(ConnectionStatus.DISCONNECTED);
      setConnection(null);
      setPublishTopic(null);
    });
  };

  const createPublishTopic = () => {
    if (!connection) {
      return;
    }

    const cmdVel = new Topic({
      ros: connection,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    setPublishTopic(cmdVel);
  };

  const connect = (url: string): boolean => {
    setConnectionState(ConnectionStatus.CONNECTING);
    try {
      const ros = new Ros({url});

      setConnection(ros);

      return true;
    } catch (e) {
      console.log(e);
      setConnectionState(ConnectionStatus.FAILED);
      return false;
    }
  };

  const disconnect = () => {
    if (!connection) {
      return;
    }
    connection.close();
    setConnection(null);
    setPublishTopic(null);
  }

  useEffect(() => {

    if (connection) {
      createListener();
    }
  }, [connection]);

  const sendMovement = (x: number, y: number, z: number, th: number) => {
    if (!publishTopic) {
      throw new Error('Publish topic is not set');
    }
    publishTopic.publish({
      linear: {
        x,
        y,
        z,
      },
      angular: {
        x: 0,
        y: 0,
        z: th,
      },
    });
  };
  const startMovementInterval = (movement: MovementDirection) => {
    if (intervalId > 0) {
      return;
    }

    const id: number = setInterval(() => {
      const coords = MOVEMENTS[movement];
      sendMovement(coords.x, coords.y, coords.z, coords.th);
    }, 100) as unknown as number;

    setIntervalId(id);
  };

  const stopMovement = () => {
    clearInterval(intervalId);
    setIntervalId(0);
  };

  return {
    connectionState,
    connect,
    startMovementInterval,
    stopMovement,
    disconnect
  }
};
