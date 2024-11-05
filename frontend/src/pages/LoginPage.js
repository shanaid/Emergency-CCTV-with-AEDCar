import "../styles/LoginPage.css";
import LoginForm from "../components/Loginform";

function LoginPage() {
  return (
    <div className="login-container">
      <div className="loginbox">
        <h1 className="title">Login</h1>
        <LoginForm />
      </div>
    </div>
  );
}

export default LoginPage;
