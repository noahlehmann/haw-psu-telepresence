import React, { useState } from 'react';
import {
  Alert,
  AlertIcon,
  AlertText,
  Box,
  Button,
  ButtonIcon,
  Center,
  ChevronDownIcon,
  ChevronLeftIcon,
  ChevronRightIcon,
  ChevronUpIcon,
  HStack,
  Input,
  InputField,
} from '@gluestack-ui/themed';
import GuestLayout from '../../layouts/GuestLayout';
import { InfoIcon } from 'lucide-react-native';
import { Text } from 'react-native';
import { Ros, Topic } from 'roslib';


const AlertBox = ({ text, action }: { text: string, action: 'error'|'info'|'warning' }) => (
  <Alert mx='$2.5' action={action} variant='solid' w='$full'>
    <AlertIcon as={InfoIcon} mr='$3' />
    <AlertText>
      {text}
    </AlertText>
  </Alert>
);

const enum ConnectionStatus {
  CONNECTED,
  DISCONNECTED,
  CONNECTING,
  FAILED,
}


export default function SplashScreen () {
  const ros = new Ros({ url : 'ws://192.168.117.131:9090' });

  const cmdVel = new Topic({
                             ros,
                             name        : '/cmd_vel',
                             messageType : 'geometry_msgs/Twist',
                           });

  const MOVEMENTS = {
    FORWARD  : {
      x  : 1,
      y  : 0,
      z  : 0,
      th : 0,
    },
    BACKWARD : {
      x  : -1,
      y  : 0,
      z  : 0,
      th : 0,
    },
    LEFT     : {
      x  : 0,
      y  : 0,
      z  : 0,
      th : 1,

    },
    RIGHT    : {
      x  : 0,
      y  : 0,
      z  : 0,
      th : -1,

    },

  };


  // Zustandsvariablen
  const [inputValue, setInputValue]             = useState('192.168.117.131');
  const [isInputInvalid, setIsInputInvalid]     = useState(false);
  const [connectionStatus, setConnectionStatus] = useState(ConnectionStatus.DISCONNECTED);
  const connectRobot                            = () => {
    if (!validateInput(inputValue)) {
      setIsInputInvalid(true);
      return;
    }
    setConnectionStatus(ConnectionStatus.FAILED);
  };
  const validateInput                           = (value: string) => {
    const regex = /^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    return regex.test(value);
  };

  const sendMovement = (x: number, y: number, z: number, th: number) => {
    cmdVel.publish({
                     linear  : {
                       x,
                       y,
                       z,
                     },
                     angular : {
                       x : 0,
                       y : 0,
                       z : th,
                     },
                   });
  };


  const [intervalId, setIntervalId] = useState(0);
  const startMovementInterval       = (movement: keyof typeof MOVEMENTS) => {
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
    sendMovement(0, 0, 0, 0);
  };

  const handleInputChange = (value: string) => {
    setInputValue(value);

    // Validierung
    if (!validateInput(value)) {
      setIsInputInvalid(true);
    } else {
      setIsInputInvalid(false);
    }
  };
  return (
    <GuestLayout>

      <Center w='$full' flex={1}>
        <Input
          w='$full'
          variant='outline'
          size='xl'

          isDisabled={false}
          isInvalid={isInputInvalid}
          isReadOnly={false}
        >
          <InputField value={inputValue} onChangeText={handleInputChange} placeholder='Enter Robot IP here' />

        </Input>
        {
          isInputInvalid && <AlertBox action='error' text='Invalid IP' />
        }
        {
          connectionStatus === ConnectionStatus.FAILED && <AlertBox action='error' text='Connection failed' />
        }
        {
          connectionStatus === ConnectionStatus.CONNECTING && <AlertBox action='warning' text='Connecting..' />
        }
        {
          connectionStatus === ConnectionStatus.CONNECTED && <AlertBox action='info' text='Connected' />
        }
        <Button w='$full' onPress={connectRobot}>
          <Text>
            Connect
          </Text>
        </Button>
      </Center>
      <Center w='$full' flex={1}>
        <HStack reversed={false}>
          <Box w='$20' h='$20' />
          <Button
            onPressIn={() => startMovementInterval('FORWARD')}
            onPressOut={stopMovement}
            w='$20' h='$20'
            borderRadius='$full'
            size='lg'
            p='$3.5'
            bg='$indigo600'
            borderColor='$indigo600'
          >
            <ButtonIcon as={ChevronUpIcon} />
          </Button>
          <Box w='$20' h='$20' />
        </HStack>
        <HStack reversed={false}>
          <Button
            onPressIn={() => startMovementInterval('LEFT')}
            onPressOut={stopMovement}
            w='$20' h='$20'
            borderRadius='$full'
            size='lg'
            p='$3.5'
            bg='$indigo600'
            borderColor='$indigo600'
          >
            <ButtonIcon as={ChevronLeftIcon} />
          </Button>
          <Box w='$20' h='$20' /><Button
          onPressIn={() => startMovementInterval('RIGHT')}
          onPressOut={stopMovement}
          w='$20' h='$20'
          borderRadius='$full'
          size='lg'
          p='$3.5'
          bg='$indigo600'
          borderColor='$indigo600'
        >
          <ButtonIcon as={ChevronRightIcon} />
        </Button>
        </HStack>
        <HStack reversed={false}>
          <Box w='$20' h='$20' />
          <Button
            onPressIn={() => startMovementInterval('BACKWARD')}
            onPressOut={stopMovement}
            w='$20' h='$20'
            borderRadius='$full'
            size='lg'
            p='$3.5'
            bg='$indigo600'
            borderColor='$indigo600'
          >
            <ButtonIcon as={ChevronDownIcon} />
          </Button>
          <Box w='$20' h='$20' />
        </HStack>
      </Center>
    </GuestLayout>
  );
}
