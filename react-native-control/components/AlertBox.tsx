import {Alert, AlertIcon, AlertText} from "@gluestack-ui/themed";
import {InfoIcon} from "lucide-react-native";
import React from "react";

interface AlertBoxProps {
  text: string;
  action: 'error' | 'info' | 'warning';
}

export const AlertBox = ({text, action}: AlertBoxProps) => (
  <Alert mx="$2.5" action={action} variant="solid" w="$full">
    <AlertIcon as={InfoIcon} mr="$3"/>
    <AlertText>
      {text}
    </AlertText>
  </Alert>
);
