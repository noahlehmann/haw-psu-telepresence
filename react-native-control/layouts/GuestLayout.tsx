import React from 'react';
import { Box, ScrollView, StatusBar, VStack } from '@gluestack-ui/themed';

import { KeyboardAwareScrollView } from 'react-native-keyboard-aware-scroll-view';
import { SafeAreaView } from 'react-native-safe-area-context';


type GuestLayoutProps = {
  children: React.ReactNode;
};

export default function GuestLayout(props: GuestLayoutProps) {
  return (
    <SafeAreaView style={{ flex: 1 }}>
      <KeyboardAwareScrollView contentContainerStyle={{ flexGrow: 1 }}>
        <Box
          sx={{
            _web: {
              height: '100vh',
              overflow: 'hidden',
            },
          }}
          height="100%"
        >
          <StatusBar
            translucent
            backgroundColor="transparent"
            barStyle="light-content"
          />
          <ScrollView
            flex={1}
            contentContainerStyle={{
              alignItems: 'center',
              flexGrow: 1,
              justifyContent: 'center',
            }}
            sx={{
              '@base' : { _light : { bg : '$backgroundLight0' } },
              '@md'   : { _light : { bg : '$backgroundLight0' }, p : '$8' },
              '_dark': { bg: '$backgroundDark900' },
            }}
            bounces={false}
          >
            <VStack
              w="$full"
              flex={1}
              overflow="hidden"
              sx={{
                '@md': {
                  maxWidth: '$containerWidth',
                  flexDirection: 'row',
                  rounded: '$xl',
                  flex: undefined,
                },
              }}
            >
              {props.children}
            </VStack>
          </ScrollView>
        </Box>
      </KeyboardAwareScrollView>
    </SafeAreaView>
  );
}
