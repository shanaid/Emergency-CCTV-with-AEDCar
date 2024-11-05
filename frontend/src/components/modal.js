// components/Modal.js (수정)
import "../styles/Modal.css";

const Modal = ({ isOpen, onClose, title, content, children }) => {
  if (!isOpen) return null;

  return (
    <div className="modalbackdrop" onClick={onClose}>
      <div
        className="modalcontent"
        onClick={(e) => e.stopPropagation()} // 이벤트 버블링 방지
      >
        <h2>{title}</h2>
        <p>{content}</p>
        {children}
      </div>
    </div>
  );
};

export default Modal;
