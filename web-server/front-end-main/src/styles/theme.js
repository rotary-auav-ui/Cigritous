import { createTheme } from "@mui/material";

const theme = createTheme({
  palette: {
    primary: { main: "#3A3A3A", contrastText: "#FFFFFF" },
    secondary: { main: "#000000", contrastText: "#FFFFFF" },
    background: {
      default: "#262626",
    },
    text: {
      primary: "#FFFFFF",
    },
  },
  typography: {
    allVariants: {
      fontFamily: "Robot,poppins",
      textTransform: "none",
    },
  },
  components: {
    MuiLink: {
      styleOverrides: {
        root: {
          textDecoration: "none",
          fontWeight: 700,
        },
      },
    },
    MuiCard: {
      styleOverrides: {
        root: {
          display: "flex",
          flexDirection: "column",
          justifyContent: "space-between",
        },
      },
    },
  },
});

export default theme;
