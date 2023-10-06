# Telepresence React Native Control App

## Quick Starter

Run the app with npm start or yarn start. This will start the Expo development server and print a QR code in your terminal.

## Architecture

The app is built with React Native and Expo. The UI is built with gluestack-ui. The app is built with TypeScript.
In the ui you can insert the IP Address of the telepresence robot. The port being used is 9090.
This needs to be adjusted if needed inside the code.
The app then sends ROS messages to the robot to control it via the websocket protocol.
This implementation is based on the [ROS2 Control Teleop](https://wiki.ros.org/roslibjs)

This software is built above a starter kit from Gluestack.io. The original README is below.

Important Folders:
- screens/Login/SplashScreen.tsx - Contains the main view that shows the control buttons and input
- hooks/ros-hook.ts - Contains the code that connects to the robot and sends the messages

# Expo gluestack-ui Head Starter Kit 🚀

A comprehensive starter kit to kick-start your next mobile application using [Expo](https://expo.io/) and [gluestack-ui](https://ui.gluestack.io) - your one-stop solution for faster, smoother, and better mobile and web development.

## 👩‍💻 Built with

- [gluestack-ui](https://ui.gluestack.io)
- [Expo](https://expo.io/)
- [Expo Router V2](https://blog.expo.dev/introducing-expo-router-v2-3850fd5c3ca1)
- [TypeScript](https://www.typescriptlang.org/)

## 🌟 Features

- **Seamless Integration**: Get up and running with Expo and gluestack-ui in no time.
- **Modular Architecture**: Built with scalability in mind to cater to both small and large projects.
- **Preconfigured Navigation**: Includes preconfigured navigation patterns using [Expo Router V2](https://blog.expo.dev/introducing-expo-router-v2-3850fd5c3ca1).
- **Theming & Styling**: Customizable themes using gluestack-ui components.
- **Detailed Documentation**: Comprehensive documentation to guide you through every step.
- **Extensive form handling**: Extensive implementation of handling forms and validations.
- **Preconfigured linting rules**: Comprehensive linting rules.
- **Preconfigured editor configurations**: Widely accepted editor configuration.

## 📷 Screenshots

Please find the screenshots in the `head-starter-kit-screenshots` folder.

## 🚀 Getting Started

### Prerequisites

- [Node.js](https://nodejs.org/)
- [Expo CLI](https://expo.io/tools#cli)

### Installation

```bash
# Clone the repository

## Using SSH
git clone git@github.com:gluestack/expo-head-starter-kit.git

## Using HTTPS
git clone https://github.com/gluestack/expo-head-starter-kit.git

# Navigate into the directory
cd expo-head-starter-kit

# Install dependencies
npm install

# Start the Expo development server
expo start
```

## 🌈 Applying Fonts in your application

You already have the font loaded in your application, now you can apply the font in your application.

Add the font name in the `gluestack.config.js` file in the `fonts` object.

```javascript
fonts: {
  heading: "inter",
  body: "inter",
  mono: "monospace",
},
```

## 📖 Documentation

For a deep dive into the components, structure, and configurations, please refer to our [detailed documentation](https://ui.gluestack.io/docs/getting-started/installation).

## 🧪 Testing

We've set up Jest as the testing framework for this project to ensure the reliability of your codebase. You can run the tests using the following commands:

### Setting Up Jest with Expo

To install `jest-expo` in your project, run the following command:

```bash
npx expo install jest-expo jest
```

### Writing Test Cases

To write your own test cases, refer to the [Expo documentation on unit testing](https://docs.expo.dev/develop/unit-testing/) for comprehensive guidance and best practices.

### Running Tests

To run tests, use the following command:

```bash
npm run test
```

We've added an example of a Jest snapshot test case for the `SplashScreen` component. This test case uses Jest's `jest.mock` to mock a dependency and then renders the `SplashScreen` component within a `StyledProvider`. Finally, it asserts that the rendered component matches the previously saved snapshot.

You can add more test cases as needed for your other components and features. This section serves as a starting point for incorporating testing into your project using Jest.

## 🙌 Contributing

We welcome contributions! Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo, make changes and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the [MIT License](https://opensource.org/license/mit/).

## ❤️ Acknowledgments

- Thanks to the [Expo team](https://expo.io/) for creating an amazing platform.
- Shoutout to [gluestack-ui contributors](https://gluestack.io/) for their outstanding work.

```

```
