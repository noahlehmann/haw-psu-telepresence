import React, {useEffect, useState} from 'react';
import {
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
import {ConnectionStatus, useRosConnect} from './ros-connect';
import {MovementDirection} from './movements';
import {Text} from 'react-native';
import {AlertBox} from '../../components/AlertBox';

export default function SplashScreen() {

  const {connect, connectionState, stopMovement, startMovementInterval, disconnect} = useRosConnect();
  const [inputValue, setInputValue] = useState('192.168.117.131');
  const [isInputInvalid, setIsInputInvalid] = useState(false);

  useEffect(
    () => {
      return () => {
        stopMovement()
        disconnect();
      }
    }, []
  )

  const connectRobot = () => {
    if (!validateInput(inputValue)) {
      setIsInputInvalid(true);
      return;
    }

    connect(`ws://${inputValue}:9090`);
  };
  const validateInput = (value: string) => {
    const regex = /^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    return regex.test(value);
  };

  const handleInputChange = (value: string) => {
    setInputValue(value);

    if (!validateInput(value)) {
      setIsInputInvalid(true);
    } else {
      setIsInputInvalid(false);
    }
  };
  return (
    <GuestLayout>
      <Center w="$full" flex={1}>
        <Input
          w="$full"
          variant="outline"
          size="xl"
          isDisabled={false}
          isInvalid={isInputInvalid}
          isReadOnly={false}
        >
          <InputField value={inputValue} onChangeText={handleInputChange} placeholder="Enter Robot IP here"/>
        </Input>
        {
          isInputInvalid && <AlertBox action="error" text="Invalid IP"/>
        }
        {
          connectionState === ConnectionStatus.FAILED && <AlertBox action="error" text="Connection failed"/>
        }
        {
          connectionState === ConnectionStatus.CONNECTING && <AlertBox action="warning" text="Connecting.."/>
        }
        {
          connectionState === ConnectionStatus.CONNECTED && <AlertBox action="info" text="Connected"/>
        }
        {
          connectionState === ConnectionStatus.CONNECTED ?
            <Button w="$full" onPress={disconnect}>
              <Text>
                Disconnect
              </Text>
            </Button> :
            <Button w="$full" onPress={connectRobot} isDisabled={connectionState === ConnectionStatus.CONNECTING}>
              <Text>
                Connect
              </Text>
            </Button>
        }
      </Center>
      <Center w="$full" flex={1}>
        <HStack reversed={false}>
          <Box w="$20" h="$20"/>
          <Button
            onPressIn={() => startMovementInterval(MovementDirection.FORWARD)}
            onPressOut={stopMovement}
            w="$20" h="$20"
            borderRadius="$full"
            size="lg"
            p="$3.5"
            bg="$indigo600"
            borderColor="$indigo600"
          >
            <ButtonIcon as={ChevronUpIcon}/>
          </Button>
          <Box w="$20" h="$20"/>
        </HStack>
        <HStack reversed={false}>
          <Button
            onPressIn={() => startMovementInterval(MovementDirection.LEFT)}
            onPressOut={stopMovement}
            w="$20" h="$20"
            borderRadius="$full"
            size="lg"
            p="$3.5"
            bg="$indigo600"
            borderColor="$indigo600"
          >
            <ButtonIcon as={ChevronLeftIcon}/>
          </Button>
          <Box w="$20" h="$20"/><Button
          onPressIn={() => startMovementInterval(MovementDirection.RIGHT)}
          onPressOut={stopMovement}
          w="$20" h="$20"
          borderRadius="$full"
          size="lg"
          p="$3.5"
          bg="$indigo600"
          borderColor="$indigo600"
        >
          <ButtonIcon as={ChevronRightIcon}/>
        </Button>
        </HStack>
        <HStack reversed={false}>
          <Box w="$20" h="$20"/>
          <Button
            onPressIn={() => startMovementInterval(MovementDirection.BACKWARD)}
            onPressOut={stopMovement}
            w="$20" h="$20"
            borderRadius="$full"
            size="lg"
            p="$3.5"
            bg="$indigo600"
            borderColor="$indigo600"
          >
            <ButtonIcon as={ChevronDownIcon}/>
          </Button>
          <Box w="$20" h="$20"/>
        </HStack>
      </Center>
    </GuestLayout>
  );
}
