import React from 'react';
import { useAuth } from '../context/AuthContext';
import Link from '@docusaurus/Link';
import { NavbarNavLink } from '@docusaurus/theme-common';

const NavbarAuth = () => {
  const { user, signOut, loading } = useAuth();

  if (loading) {
    return (
      <div className="navbar__item">
        <span>Loading...</span>
      </div>
    );
  }

  if (user) {
    // User is logged in - show avatar and dropdown
    return (
      <div className="navbar__item dropdown dropdown--right dropdown--end">
        <button 
          className="navbar__link"
          aria-haspopup="true"
          aria-expanded="false"
          aria-label="User profile"
        >
          <div className="avatar">
            <img 
              className="avatar__photo avatar__photo--sm" 
              src={`https://ui-avatars.com/api/?name=${encodeURIComponent(user.email || 'User')}&background=2e8555&color=fff`} 
              alt="User avatar" 
            />
          </div>
        </button>
        <ul className="dropdown__menu">
          <li>
            <Link 
              className="dropdown__link" 
              to="/profile"
            >
              Profile
            </Link>
          </li>
          <li>
            <button 
              className="dropdown__link dropdown__link--container"
              onClick={async () => {
                await signOut();
              }}
            >
              Logout
            </button>
          </li>
        </ul>
      </div>
    );
  } else {
    // User is not logged in - show sign in/up buttons
    return (
      <>
        <NavbarNavLink 
          to="/auth/signin"
          label="Sign In"
          position="right"
        />
        <NavbarNavLink 
          to="/auth/signup"
          label="Sign Up"
          position="right"
        />
      </>
    );
  }
};

export default NavbarAuth;