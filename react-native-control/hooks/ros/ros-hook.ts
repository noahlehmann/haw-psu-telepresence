import {useEffect, useState} from 'react';
import {Ros, Topic} from 'roslib';
import {MovementDirection, MOVEMENTS} from './movements';

export const enum ConnectionStatus {
  CONNECTED,
  DISCONNECTED,
  CONNECTING,
  FAILED,
}

/**
 * This hook is responsible for connecting to ROS server and sending movement commands.
 * It returns connection state, connect function, startMovementInterval function and stopMovement function.
 */
export const useRosConnect = () => {
  const [connection, setConnection] = useState<null | Ros>(null);
  const [publishTopic, setPublishTopic] = useState<null | Topic>(null);
  const [connectionState, setConnectionState] = useState(ConnectionStatus.DISCONNECTED);
  const [intervalId, setIntervalId] = useState(0);

  /**
   * This function creates listeners for connection, error and close events.
   * It also creates publish topic if the connection is successful.
   */
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

  /**
   * To send movement commands we need to create a publish topic. After that we can send messages to this topic.
   */
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

  /**
   * This function connects to ROS server. Returns true if connection is successful, false otherwise.
   */
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

  /**
   * This function disconnects from ROS server.
   */
  const disconnect = () => {
    if (!connection) {
      return;
    }
    connection.close();
    setConnection(null);
    setPublishTopic(null);
  }

  /**
   * This effect is responsible for creating listeners for connection, error and close events.
   */
  useEffect(() => {
    if (connection) {
      createListener();
    }
  }, [connection]);

  /**
   * This effect is responsible for sending movement commands.
   * Typically, you should use the MOVEMENTS object from movements.ts file.
   */
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
  /**
   * This function starts sending movement commands to the robot. It uses setInterval function to continuously send commands
   * and move the robot.
   */
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

  /**
   * This function stops sending movement commands to the robot.
   */
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
